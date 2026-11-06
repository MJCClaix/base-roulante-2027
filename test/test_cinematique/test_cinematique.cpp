// Tests unitaires des fonctions pures du .ino : ajustement_consigne() et calcule_consignes_moteurs().
//
// Ces tests vérifient des invariants (symétries, bornes, monotonie, proportions) et non des valeurs
// numériques liées aux réglages (RAYON_*, COEFFICIENT_REDUCTION_*, PROGRESSIVITE_*) : on doit pouvoir
// ajuster ces réglages sans casser les tests.
#include <PID_v1.h>  // inclus ici pour que PlatformIO détecte la dépendance (il ne lit pas les #include du .ino)
#include <PinChangeInterrupt.h>
#include <gtest/gtest.h>

#include "BaseMoteurHolonome.ino"

namespace {

constexpr float TOLERANCE = 1e-3f;

struct Consignes {
  float droit, gauche, arriere;
};

Consignes calculer(float x, float y, float rotation) {
  Consignes c{};
  calcule_consignes_moteurs(x, y, rotation, c.droit, c.gauche, c.arriere);
  return c;
}

float max_abs(const Consignes& c) { return max(max(fabs(c.droit), fabs(c.gauche)), fabs(c.arriere)); }

// ------------------------------------------------------------------------------------------------
// ajustement_consigne

TEST(AjustementConsigne, ZeroResteZero) { EXPECT_NEAR(ajustement_consigne(0), 0, TOLERANCE); }

TEST(AjustementConsigne, PleineEchelleInchangee) {
  EXPECT_NEAR(ajustement_consigne(CONSIGNE_MAX), CONSIGNE_MAX, TOLERANCE);
  EXPECT_NEAR(ajustement_consigne(-CONSIGNE_MAX), -CONSIGNE_MAX, TOLERANCE);
}

TEST(AjustementConsigne, FonctionImpaire) {
  for (float c = 0; c <= CONSIGNE_MAX; c += 5) {
    EXPECT_NEAR(ajustement_consigne(-c), -ajustement_consigne(c), TOLERANCE) << "c=" << c;
  }
}

TEST(AjustementConsigne, StrictementCroissante) {
  float precedente = ajustement_consigne(-CONSIGNE_MAX);
  for (float c = -CONSIGNE_MAX + 5; c <= CONSIGNE_MAX; c += 5) {
    const float courante = ajustement_consigne(c);
    EXPECT_GT(courante, precedente) << "c=" << c;
    precedente = courante;
  }
}

TEST(AjustementConsigne, AdoucitLaCommandeAutourDuCentre) {
  // La courbe ne doit jamais dépasser la réponse linéaire (en valeur absolue).
  for (float c = 5; c < CONSIGNE_MAX; c += 5) {
    EXPECT_LE(fabs(ajustement_consigne(c)), c + TOLERANCE) << "c=" << c;
  }
}

// ------------------------------------------------------------------------------------------------
// calcule_consignes_moteurs

TEST(CalculeConsignesMoteurs, EntreesNullesDonnentConsignesNulles) {
  const Consignes c = calculer(0, 0, 0);
  EXPECT_NEAR(c.droit, 0, TOLERANCE);
  EXPECT_NEAR(c.gauche, 0, TOLERANCE);
  EXPECT_NEAR(c.arriere, 0, TOLERANCE);
}

TEST(CalculeConsignesMoteurs, AxeX_RoueDroiteEtGaucheOpposeesRoueArriereNulle) {
  const Consignes c = calculer(20, 0, 0);
  EXPECT_NEAR(c.droit, -c.gauche, TOLERANCE);
  EXPECT_GT(c.gauche, 0);
  EXPECT_NEAR(c.arriere, 0, TOLERANCE);
}

TEST(CalculeConsignesMoteurs, AxeY_RoueDroiteEtGaucheEgalesRoueArriereOpposee) {
  const Consignes c = calculer(0, 20, 0);
  EXPECT_NEAR(c.droit, c.gauche, TOLERANCE);
  EXPECT_GT(c.droit, 0);
  EXPECT_LT(c.arriere, 0);
  EXPECT_NEAR(c.arriere, -2 * c.droit, TOLERANCE);  // la somme des vitesses de translation est nulle
}

TEST(CalculeConsignesMoteurs, RotationPure_TroisRouesDansLeMemeSens) {
  const Consignes c = calculer(0, 0, 5);
  EXPECT_GT(c.droit, 0);
  EXPECT_NEAR(c.droit, c.gauche, TOLERANCE);
  EXPECT_NEAR(c.droit, c.arriere, TOLERANCE);
}

TEST(CalculeConsignesMoteurs, Antisymetrie_InverserLesEntreesInverseLesSorties) {
  const Consignes a = calculer(12, -7, 3);
  const Consignes b = calculer(-12, 7, -3);
  EXPECT_NEAR(a.droit, -b.droit, TOLERANCE);
  EXPECT_NEAR(a.gauche, -b.gauche, TOLERANCE);
  EXPECT_NEAR(a.arriere, -b.arriere, TOLERANCE);
}

TEST(CalculeConsignesMoteurs, Superposition_HorsSaturationLaCinematiqueEstLineaire) {
  const Consignes x = calculer(8, 0, 0);
  const Consignes y = calculer(0, 6, 0);
  const Consignes r = calculer(0, 0, 2);
  const Consignes somme = calculer(8, 6, 2);
  ASSERT_LT(max_abs(somme), VITESSE_MAX_MOTEURS) << "entrées trop grandes pour ce test";
  EXPECT_NEAR(somme.droit, x.droit + y.droit + r.droit, TOLERANCE);
  EXPECT_NEAR(somme.gauche, x.gauche + y.gauche + r.gauche, TOLERANCE);
  EXPECT_NEAR(somme.arriere, x.arriere + y.arriere + r.arriere, TOLERANCE);
}

TEST(CalculeConsignesMoteurs, ConsignesToujoursBorneesParLaVitesseMax) {
  for (float x = -CONSIGNE_MAX; x <= CONSIGNE_MAX; x += 51) {
    for (float y = -CONSIGNE_MAX; y <= CONSIGNE_MAX; y += 51) {
      for (float r = -CONSIGNE_MAX; r <= CONSIGNE_MAX; r += 51) {
        EXPECT_LE(max_abs(calculer(x, y, r)), VITESSE_MAX_MOTEURS + TOLERANCE)
            << "x=" << x << " y=" << y << " r=" << r;
      }
    }
  }
}

TEST(CalculeConsignesMoteurs, SaturationConserveLesProportionsEntreRoues) {
  // Même direction demandée avec une faible amplitude (non saturée) puis avec une forte (saturée) :
  // la normalisation ramène la roue la plus rapide à VITESSE_MAX_MOTEURS sans déformer la trajectoire.
  const Consignes faible = calculer(5, 4, 2);
  const Consignes forte = calculer(255, 204, 102);  // 51 fois plus grand
  ASSERT_GT(max_abs(faible), 0);
  ASSERT_LT(max_abs(faible), VITESSE_MAX_MOTEURS) << "l'entrée faible ne doit pas saturer";
  ASSERT_GT(51 * max_abs(faible), VITESSE_MAX_MOTEURS) << "l'entrée forte doit saturer";
  const float facteur = VITESSE_MAX_MOTEURS / max_abs(faible);
  EXPECT_NEAR(forte.droit, faible.droit * facteur, 0.01f * VITESSE_MAX_MOTEURS);
  EXPECT_NEAR(forte.gauche, faible.gauche * facteur, 0.01f * VITESSE_MAX_MOTEURS);
  EXPECT_NEAR(forte.arriere, faible.arriere * facteur, 0.01f * VITESSE_MAX_MOTEURS);
}

TEST(CalculeConsignesMoteurs, PleinAvantSature_LaRoueLaPlusRapideAtteintLaVitesseMax) {
  EXPECT_NEAR(max_abs(calculer(CONSIGNE_MAX, 0, 0)), VITESSE_MAX_MOTEURS, TOLERANCE);
  EXPECT_NEAR(max_abs(calculer(0, CONSIGNE_MAX, 0)), VITESSE_MAX_MOTEURS, TOLERANCE);
  EXPECT_NEAR(max_abs(calculer(0, 0, CONSIGNE_MAX)), VITESSE_MAX_MOTEURS, TOLERANCE);
}

}  // namespace

#include "lanceur_gtest.h"
