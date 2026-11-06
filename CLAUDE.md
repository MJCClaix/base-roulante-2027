# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Nature du projet

Firmware Arduino (C++) pour une base roulante holonome à 3 roues motorisées (2027). Le firmware se compile et se téléverse depuis l'IDE Arduino (pas de build ni de linter dans le dépôt). Des tests tournent sur PC avec PlatformIO (environnement `native`, googletest/googlemock), voir la section Commandes. Le matériel visé est un shield moteur L298P KeyStudio ; les broches A0/A1 utilisées pour les codeurs indiquent une carte de type Arduino Uno/Nano (non vérifié dans le dépôt).

Bibliothèques externes requises (à installer dans l'IDE, non versionnées ici) :
- `PinChangeInterrupt` (interruptions sur broches quelconques, `attachPinChangeInterrupt`)
- `ServoInput` (lecture des PWM du récepteur de télécommande, `ServoInputPin`)
- `PID_v1` (régulateur PID, classe `PID`)

Les commentaires et les noms de variables sont en français : conserver cette convention.

## Commandes

- Compilation / téléversement : via l'IDE Arduino ouvrant `BaseMoteurHolonome.ino` (le `.h` doit être dans le même dossier).
- Tests : `pio test -e native` (installation : `setup_tests.ps1`). Trois suites dans `test/` : `test_cinematique`, `test_pilotage_moteur`, `test_bout_a_bout`. Les faux Arduino / ServoInput / PinChangeInterrupt et le moteur simulé sont dans `test/support/`.
- Lint : aucun. La vérification finale se fait sur le matériel (moniteur série à 115200 bauds, `DEBUG true`).

## Règles de travail

- **Tests** : à chaque fin d'ajout ou de modification de code, lancer `pio test -e native` (toutes les suites) et rapporter le résultat, sauf mention contraire de l'utilisateur.
- Les tests vérifient des propriétés (bornes, symétries, convergence), pas des valeurs liées aux réglages, pour ne pas bloquer les évolutions du comportement. Les tests nommés `ComportementActuel_*` décrivent un comportement destiné à changer.
- Conserver les fins de ligne CRLF de `BaseMoteurHolonome.ino` et `PilotageMoteur.h`.

## Architecture

Deux fichiers, un seul flux d'exécution :

- **`BaseMoteurHolonome.ino`** : point d'entrée (`setup` / `loop`) et cinématique.
  1. `loop()` lit trois canaux de télécommande (avant/arrière, droite/gauche, rotation) via `ServoInputPin`, avec zone morte (`ZONE_MORTE_TELECOMMANDE`).
  2. `ajustement_consigne()` applique une courbe non linéaire (terme cubique, `PROGRESSIVITE_TELECOMMANDE`) pour adoucir la commande autour du centre.
  3. `calcule_consignes_moteurs()` convertit (x, y, rotation) en consignes des trois moteurs (droit, gauche, arrière) avec les termes cinématiques des roues (`RAYON_ROUES`, `RAYON_PLATEFORME`, facteur 0.866 = sin 60°) puis normalise si une consigne dépasse `VITESSE_MAX_MOTEURS`.
  4. `set_vitesse()` transmet les consignes aux trois objets moteurs, puis appelle `execute()` sur chacun.

- **`PilotageMoteur.h`** : classe template `PilotageMoteur<pinCodeur>`, une instance par moteur. Le paramètre de template est la broche du codeur ; cela donne à chaque moteur ses propres membres `static` (compteur d'impulsions, durée, date) et sa propre routine d'interruption.
  - `routineInterruptionCodeur()` compte les fronts montants et mémorise la durée de `NB_PULSES_CODEUR_MOTEURS` impulsions.
  - `get_VitesseRotation()` convertit cette durée en tours/s (le rapport de réduction `11` est codé en dur, dans la macro et dans la méthode).
  - `execute()` mesure la vitesse, appelle le PID (échantillonnage 20 ms, sortie bornée 0–255) et écrit la commande PWM ; une sortie série périodique est possible en mode debug.
  - `set_ConsigneVitesse()` prend le signe de la consigne pour piloter la broche de sens, avec inversion possible via `set_InverseSensMoteur()`.
  - **Freinage par impulsion inverse** (`set_FreinageInverse()`, activé dans `setup()`) : quand la consigne passe à 0, `execute()` envoie une impulsion PWM de sens opposé, de force et durée proportionnelles à la vitesse mesurée (constantes `FREINAGE_*`, à régler sur le robot), le PID étant en manuel pendant l'impulsion. Une consigne nulle ne bascule alors plus la broche de sens. Le sens de rotation est supposé égal à celui de la dernière consigne non nulle.

- **Mapping matériel** : les trois moteurs, leurs broches de vitesse/sens (shield L298P) et leurs codeurs sont définis par `#define` en tête du `.ino`. L'inversion du sens de rotation est fixée dans `setup()` (le moteur gauche et le droit sont inversés, l'arrière non).

- **Paramètres PID** : gains passés au constructeur dans le `.ino` (`kp=5, ki=20, kd=0`). `set_ParametresPID()` existe mais n'est pas appelée.

## Points d'attention

- **Garde d'inclusion cassée** dans `PilotageMoteur.h` : `#ifndef PilotageMoteur_h` mais `#define PilotageMoteur_H` (casse différente), donc la garde ne protège rien. Sans effet tant que le fichier n'est inclus qu'une fois.
- **Fins de ligne** : les fichiers de travail sont en CRLF alors que la version committée (`HEAD`) est en LF. `git status` montre donc des diffs sur l'intégralité des fichiers ; les vraies modifications sont de faible ampleur. Ne pas convertir les fins de ligne sans demande explicite.
- **Vitesse et timeout** : le test de timeout du codeur dans `get_VitesseRotation()` est commenté ; une vitesse nulle n'est donc pas détectée quand le moteur s'arrête. Le compteur de durée est initialisé à 1 000 000 µs, ce qui donne une vitesse quasi nulle au démarrage.
- **Code de test laissé en place** : forçages de consignes et multiplications commentés dans `loop()` et `calcule_consignes_moteurs()` (`// a remettre !!!`). Vérifier ces lignes avant de tester sur le robot.
- `DEBUG` est défini à `true` dans le `.ino` : la sortie série est active par défaut et peut ralentir la boucle.
