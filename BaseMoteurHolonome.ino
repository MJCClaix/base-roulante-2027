// version 2027
// 3 moteur Holonomes
//  moteur A = gauche
//  moteur B = arriere
//  moteur C = droite
//voie    mot  validation vitesse codeur
// A       G     3        6         13
// B       AR    4        5         A0
// C       D     7        10        A1
// D             8        9

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <ServoInput.h>
#include "PilotageMoteur.h"

#define PIN_TELECOMMANDE_AV_AR    2
#define PIN_TELECOMMANDE_DR_GA    11  //  le 8 et le 9 sont utilisées par le moteur D !
#define PIN_TELECOMMANDE_ROTATION 12

#define PIN_CODEUR_MOTEUR_DROIT       15  // A1
#define PIN_CODEUR_MOTEUR_GAUCHE       13
#define PIN_CODEUR_MOTEUR_ARRIERE       14  // A0

// Brochage des commandes de moteurs imposé par le shield L298P KeyStudio
#define PIN_VITESSE_MOTEUR_DROIT 10
#define PIN_SENS_MOTEUR_DROIT     7
#define PIN_VITESSE_MOTEUR_GAUCHE  6
#define PIN_SENS_MOTEUR_GAUCHE     3
#define PIN_VITESSE_MOTEUR_ARRIERE  5
#define PIN_SENS_MOTEUR_ARRIERE     4

// -------------------------------------- Gestion des consignes -------------------------------------------------
#define PULSE_MINIMUM              850  // Durée minimum des pulses du récepteur de télécommande (µs)
#define PULSE_MAXIMUM              2150  // Durée maximum des pulses du récepteur de télécommande (µs)
#define CONSIGNE_MAX               255  // Valeur de consigne maximum fournie par la télécommande

// -------------------------------------- Gestion de la télécommande -------------------------------------------------
#define ZONE_MORTE_TELECOMMANDE    0.05   // Ratio de la pleine échelle en dessous duquel on ne bouge pas
#define PROGRESSIVITE_TELECOMMANDE 2

#define RAYON_ROUES       45.0
#define RAYON_PLATEFORME  120.0  // Distance des roues au centre de la plateforme (supposé le même pour les 3 roues)

// -------------------------------------- Gestion des moteurs --------------------------------------------------------
#define VITESSE_MAX_MOTEURS           255.0
// Coefficients appliqués aux consignes de la télécommande pour ajuster la sensibilité
#define COEFFICIENT_REDUCTION_AV_AR   1.0
#define COEFFICIENT_REDUCTION_DR_GA   1.0
#define COEFFICIENT_REDUCTION_ROT     0.01

#define DEBUG true

ServoInputPin<PIN_TELECOMMANDE_AV_AR>     commande_av_ar(PULSE_MINIMUM, PULSE_MAXIMUM);
ServoInputPin<PIN_TELECOMMANDE_DR_GA>     commande_dr_ga(PULSE_MINIMUM, PULSE_MAXIMUM);
ServoInputPin<PIN_TELECOMMANDE_ROTATION>  commande_rotation(PULSE_MINIMUM, PULSE_MAXIMUM);

PilotageMoteur<PIN_CODEUR_MOTEUR_DROIT> moteur_droit(PIN_VITESSE_MOTEUR_DROIT, PIN_SENS_MOTEUR_DROIT, 5, 20, 0);
PilotageMoteur<PIN_CODEUR_MOTEUR_GAUCHE> moteur_gauche(PIN_VITESSE_MOTEUR_GAUCHE, PIN_SENS_MOTEUR_GAUCHE, 5, 20, 0);
PilotageMoteur<PIN_CODEUR_MOTEUR_ARRIERE> moteur_arriere(PIN_VITESSE_MOTEUR_ARRIERE, PIN_SENS_MOTEUR_ARRIERE, 5, 20, 0);

//*******************************************************************************************************************

float ajustement_consigne(float consigne) {
  /**
   * Sert a ajuster les consignes de manière a ce que la commande de type exponentielle et non linéaire.
   */
  return (consigne + (PROGRESSIVITE_TELECOMMANDE * (pow(consigne, 3) / pow(CONSIGNE_MAX, 2)))) / (1 + PROGRESSIVITE_TELECOMMANDE);
}

//--------------------------------------------------------------------------------------------------------------------

void calcule_consignes_moteurs(
  float consigne_x, float consigne_y, float consigne_rotation,
  float &consigne_moteur_droit, float &consigne_moteur_gauche, float &consigne_moteur_arriere)
{
  /**
   * Fait la onversion des consignes du pilotes en consigne moteurs
   */

   // Conversion des commandes RC en échelle adaptée à la cinématique
  consigne_x *= RAYON_ROUES * VITESSE_MAX_MOTEURS / (0.866 * CONSIGNE_MAX);
  consigne_y *= RAYON_ROUES * VITESSE_MAX_MOTEURS / (0.866 * CONSIGNE_MAX);
  consigne_rotation *= RAYON_ROUES * VITESSE_MAX_MOTEURS / (0.866 * CONSIGNE_MAX);

  float terme_rotation = COEFFICIENT_REDUCTION_ROT * RAYON_PLATEFORME * consigne_rotation / RAYON_ROUES;
  float terme_x   = 0.866 * consigne_x / RAYON_ROUES;
  float terme_y1  = consigne_y * 0.5 / RAYON_ROUES;
  float terme_y2  = terme_y1 * 2;

  consigne_moteur_droit   = terme_rotation - terme_x + terme_y1;
  consigne_moteur_gauche  = terme_rotation + terme_x + terme_y1;
  consigne_moteur_arriere = terme_rotation - terme_y2;

  // Normalisation si dépassement de la vitesse max
  float consigne_moteurs_max = max(max(abs(consigne_moteur_droit), abs(consigne_moteur_gauche)), abs(consigne_moteur_arriere));
  if (consigne_moteurs_max > VITESSE_MAX_MOTEURS) {
    float correction = VITESSE_MAX_MOTEURS / consigne_moteurs_max;
    consigne_moteur_droit *= correction;
    consigne_moteur_gauche *= correction;
    consigne_moteur_arriere *= correction;
  }
// pour test
 //   consigne_moteur_droit *= 10;
  //  consigne_moteur_gauche *= 10;
 // consigne_moteur_arriere *= 10;



  
}

//--------------------------------------------------------------------------------------------------------------------

void set_vitesse(int vitesse_moteur_droit, int vitesse_moteur_gauche, int vitesse_moteur_arriere) {
  moteur_droit.set_ConsigneVitesse(vitesse_moteur_droit);
  moteur_gauche.set_ConsigneVitesse(vitesse_moteur_gauche);
  moteur_arriere.set_ConsigneVitesse(vitesse_moteur_arriere);

  moteur_droit.execute();
  moteur_gauche.execute();
  moteur_arriere.execute();
}

//----------------------------------------------- SETUP --------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TELECOMMANDE_AV_AR,     INPUT);
  pinMode(PIN_TELECOMMANDE_DR_GA,     INPUT);
  pinMode(PIN_TELECOMMANDE_ROTATION,  INPUT);
  
  if (DEBUG) Serial.print("Configuration des interruptions de la télécommande ... ");
  
  moteur_gauche.set_InverseSensMoteur(true);

  commande_av_ar.attach();
  commande_dr_ga.attach();
  commande_rotation.attach();
  
  if (DEBUG) Serial.println("OK");

  pinMode(PIN_CODEUR_MOTEUR_DROIT,        INPUT_PULLUP);
  pinMode(PIN_VITESSE_MOTEUR_DROIT,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_DROIT,          OUTPUT);
  
  pinMode(PIN_CODEUR_MOTEUR_GAUCHE,        INPUT_PULLUP);
  pinMode(PIN_VITESSE_MOTEUR_GAUCHE,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_GAUCHE,          OUTPUT);
  
  pinMode(PIN_CODEUR_MOTEUR_ARRIERE,        INPUT_PULLUP);
  pinMode(PIN_VITESSE_MOTEUR_ARRIERE,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_ARRIERE,          OUTPUT);

  analogWrite(PIN_VITESSE_MOTEUR_DROIT, 0);
  analogWrite(PIN_VITESSE_MOTEUR_GAUCHE, 0);
  analogWrite(PIN_VITESSE_MOTEUR_ARRIERE, 0);

  //moteur_gauche.set_InverseSensMoteur(true);
  moteur_droit.set_InverseSensMoteur(true);
  //moteur_arriere.set_InverseSensMoteur(true);

  moteur_droit.set_FreinageInverse(true);
  moteur_gauche.set_FreinageInverse(true);
  moteur_arriere.set_FreinageInverse(true);

  if (DEBUG) Serial.println("Initialisation OK !");
}

//----------------------------------------------- LOOP ---------------------------------------------------------------

void loop() {
  // Mise à l'échelle des consignes
  float consigne_x         = commande_av_ar.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
  float consigne_y         = commande_dr_ga.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
  float consigne_rotation  = commande_rotation.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
  if (DEBUG) {
    Serial.print("Consignes RC: ");
    Serial.print(consigne_x); Serial.print(", ");
    Serial.print(consigne_y); Serial.print(", ");
    Serial.print(consigne_rotation); Serial.print(", ");
  }

  
  // forcages pour tests
  //consigne_x = 0;
  //consigne_y = 0;
  //consigne_rotation = 255;

// a remettre  !!!
  consigne_x = ajustement_consigne(consigne_x);
  consigne_y = ajustement_consigne(consigne_y);
  consigne_rotation = ajustement_consigne(consigne_rotation);

  float consigne_moteur_droit, consigne_moteur_gauche, consigne_moteur_arriere;
  calcule_consignes_moteurs(
    consigne_x, consigne_y, consigne_rotation,
    consigne_moteur_droit, consigne_moteur_gauche, consigne_moteur_arriere
  );
  
  if (DEBUG) {
    Serial.print("Consignes moteurs: ");
    Serial.print(consigne_moteur_droit); Serial.print(", ");
    Serial.print(consigne_moteur_gauche); Serial.print(", ");
    Serial.print(consigne_moteur_arriere); Serial.print(", ");

    Serial.print("vitesses moteurs: ");
    Serial.print(moteur_droit.get_VitesseRotation());Serial.print(", ");
    Serial.print(moteur_gauche.get_VitesseRotation());Serial.print(", ");
    Serial.print(moteur_arriere.get_VitesseRotation());Serial.println(", ");

    
  }
  set_vitesse(consigne_moteur_droit, consigne_moteur_gauche, consigne_moteur_arriere);
}
