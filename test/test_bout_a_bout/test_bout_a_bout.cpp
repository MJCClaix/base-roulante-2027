// Tests de bout en bout : impulsions du récepteur RC -> loop() du .ino -> cinématique -> PID ->
// broches PWM / sens -> moteurs et codeurs simulés -> vitesse mesurée -> PID.
//
// On observe les sorties de la boucle de régulation (PWM écrits, vitesse réelle des roues), pas
// seulement les consignes envoyées. Les valeurs attendues sont déduites de la chaîne de calcul du .ino
// elle-même (ajustement_consigne + calcule_consignes_moteurs) : les réglages (progressivité, rayons,
// gains PID...) peuvent donc évoluer sans casser ces tests.
#include <PID_v1.h>  // inclus ici pour que PlatformIO détecte la dépendance (il ne lit pas les #include du .ino)
#include <PinChangeInterrupt.h>
#include <ServoInput.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "BaseMoteurHolonome.ino"
#include "simulation.h"

namespace {

using arduino_fake::etat;
using simulation::MoteurSimule;

constexpr unsigned long PULSE_NEUTRE = (PULSE_MINIMUM + PULSE_MAXIMUM) / 2;

// Câblage supposé des moteurs : setup() inverse le sens logiciel des moteurs droit et gauche, ce qui
// signifie qu'ils sont câblés en miroir du moteur arrière. Avec ces polarités, une consigne positive
// fait tourner chaque roue dans le sens positif de la cinématique. Si on change les inversions dans
// setup() sans changer le câblage, ces tests le détectent.
constexpr int POLARITE_DROIT = -1;
constexpr int POLARITE_GAUCHE = -1;
constexpr int POLARITE_ARRIERE = +1;

struct Consignes {
  float droit, gauche, arriere;
  float max_abs() const { return max(max(fabs(droit), fabs(gauche)), fabs(arriere)); }
};

struct Pulses {
  unsigned long av_ar = PULSE_NEUTRE;
  unsigned long dr_ga = PULSE_NEUTRE;
  unsigned long rotation = PULSE_NEUTRE;
};

class BaseRoulanteTest : public ::testing::Test {
 protected:
  BaseRoulanteTest()
      : droit(PIN_VITESSE_MOTEUR_DROIT, PIN_SENS_MOTEUR_DROIT, PIN_CODEUR_MOTEUR_DROIT, POLARITE_DROIT, 8.0),
        gauche(PIN_VITESSE_MOTEUR_GAUCHE, PIN_SENS_MOTEUR_GAUCHE, PIN_CODEUR_MOTEUR_GAUCHE, POLARITE_GAUCHE, 8.0),
        arriere(PIN_VITESSE_MOTEUR_ARRIERE, PIN_SENS_MOTEUR_ARRIERE, PIN_CODEUR_MOTEUR_ARRIERE, POLARITE_ARRIERE,
                8.0) {}

  void SetUp() override {
    etat.observateur = nullptr;
    etat.reinitialiser_broches();
    setup();  // démarrage du firmware
    remettre_regulation_a_zero();
    etat.historique_analogique.clear();
  }

  // Les moteurs du .ino sont des objets globaux : leur PID (terme intégral) et les compteurs de codeur
  // (membres static) gardent l'état du test précédent. On les ramène à un état de démarrage :
  //  1. vitesse mesurée énorme + consigne nulle => le terme intégral retombe à 0 ;
  //  2. vitesse mesurée très faible, comme au démarrage.
  // 2 * NB impulsions garantissent qu'au moins une mesure de durée complète repose sur des impulsions
  // régulières, quel que soit l'état du compteur d'impulsions laissé par le test précédent.
  void remettre_regulation_a_zero() {
    piloter(Pulses{});
    for (uint8_t codeur : {PIN_CODEUR_MOTEUR_DROIT, PIN_CODEUR_MOTEUR_GAUCHE, PIN_CODEUR_MOTEUR_ARRIERE}) {
      simulation::imposer_vitesse_mesuree(codeur, 1000.0, 2 * NB_PULSES_CODEUR_MOTEURS);
    }
    for (int i = 0; i < 50; ++i) {
      simulation::avancer_horloge(simulation::PAS_US);
      loop();
    }
    for (uint8_t codeur : {PIN_CODEUR_MOTEUR_DROIT, PIN_CODEUR_MOTEUR_GAUCHE, PIN_CODEUR_MOTEUR_ARRIERE}) {
      simulation::imposer_vitesse_mesuree(codeur, 0.5, 2 * NB_PULSES_CODEUR_MOTEURS);
    }
  }

  void piloter(const Pulses& p) {
    commande_av_ar.simuler_impulsion_us(p.av_ar);
    commande_dr_ga.simuler_impulsion_us(p.dr_ga);
    commande_rotation.simuler_impulsion_us(p.rotation);
  }

  // Consignes moteurs que la chaîne du .ino calcule pour la position actuelle des manches.
  Consignes consignes_attendues() {
    const float x = commande_av_ar.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
    const float y = commande_dr_ga.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
    const float r = commande_rotation.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
    Consignes c{};
    calcule_consignes_moteurs(ajustement_consigne(x), ajustement_consigne(y), ajustement_consigne(r), c.droit,
                              c.gauche, c.arriere);
    return c;
  }

  // Un pas de 1 ms : le temps passe, les moteurs physiques réagissent aux PWM, puis le firmware boucle.
  void pas() {
    simulation::avancer_horloge(simulation::PAS_US);
    droit.avancer(simulation::PAS_US);
    gauche.avancer(simulation::PAS_US);
    arriere.avancer(simulation::PAS_US);
    loop();
  }

  void executer_ms(int duree_ms) {
    for (int i = 0; i < duree_ms; ++i) pas();
  }

  // Adapte les moteurs simulés pour que la vitesse demandée soit atteignable (sinon le PID sature).
  void regler_moteurs_pour(const Consignes& c) {
    const double vmax = std::max(2.0 * c.max_abs(), 1.0);
    droit.regler_vitesse_max(vmax);
    gauche.regler_vitesse_max(vmax);
    arriere.regler_vitesse_max(vmax);
  }

  void verifier_vitesses(const Consignes& c) {
    // 10 % + 1 unité : set_vitesse() du .ino prend des int et tronque donc chaque consigne (jusqu'à
    // 1 unité d'écart). Cette tolérance reste valable si on corrige ce point.
    const double tolerance = 0.1 * c.max_abs() + 1.0;
    EXPECT_NEAR(droit.vitesse(), c.droit, tolerance) << "roue droite";
    EXPECT_NEAR(gauche.vitesse(), c.gauche, tolerance) << "roue gauche";
    EXPECT_NEAR(arriere.vitesse(), c.arriere, tolerance) << "roue arrière";
  }

  static void verifier_commandes_dans_les_bornes() {
    for (const auto& e : etat.historique_analogique) {
      ASSERT_GE(e.valeur, 0) << "broche " << int(e.broche);
      ASSERT_LE(e.valeur, 255) << "broche " << int(e.broche);
    }
  }

  static int pwm(uint8_t broche) { return etat.analogique[broche]; }

  MoteurSimule droit, gauche, arriere;
};

// ------------------------------------------------------------------------------------------------
// Démarrage et repos

TEST_F(BaseRoulanteTest, Setup_ConfigureLesBrochesEtLaisseLesMoteursALArret) {
  for (uint8_t broche : {PIN_VITESSE_MOTEUR_DROIT, PIN_SENS_MOTEUR_DROIT, PIN_VITESSE_MOTEUR_GAUCHE,
                         PIN_SENS_MOTEUR_GAUCHE, PIN_VITESSE_MOTEUR_ARRIERE, PIN_SENS_MOTEUR_ARRIERE}) {
    EXPECT_EQ(etat.mode[broche], OUTPUT) << "broche " << int(broche);
  }
  for (uint8_t broche : {PIN_CODEUR_MOTEUR_DROIT, PIN_CODEUR_MOTEUR_GAUCHE, PIN_CODEUR_MOTEUR_ARRIERE}) {
    EXPECT_EQ(etat.mode[broche], INPUT_PULLUP) << "broche " << int(broche);
  }
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_DROIT), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_GAUCHE), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_ARRIERE), 0);
}

TEST_F(BaseRoulanteTest, ManchesAuNeutre_AucuneCommandeMoteur) {
  piloter(Pulses{});
  executer_ms(1000);
  for (const auto& e : etat.historique_analogique) {
    ASSERT_EQ(e.valeur, 0) << "broche " << int(e.broche);
  }
  EXPECT_DOUBLE_EQ(droit.vitesse(), 0);
  EXPECT_DOUBLE_EQ(gauche.vitesse(), 0);
  EXPECT_DOUBLE_EQ(arriere.vitesse(), 0);
}

TEST_F(BaseRoulanteTest, PetitesDeviationsDansLaZoneMorte_AucuneCommandeMoteur) {
  // Zone morte : ZONE_MORTE_TELECOMMANDE * plage, répartie de part et d'autre du neutre.
  const unsigned long demi_zone = (PULSE_MAXIMUM - PULSE_MINIMUM) * ZONE_MORTE_TELECOMMANDE / 2;
  ASSERT_GT(demi_zone, 4u);
  const unsigned long deviation = demi_zone - 4;
  piloter(Pulses{PULSE_NEUTRE + deviation, PULSE_NEUTRE - deviation, PULSE_NEUTRE + deviation});
  executer_ms(1000);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_DROIT), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_GAUCHE), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_ARRIERE), 0);
}

// ------------------------------------------------------------------------------------------------
// Régulation de la vitesse des roues pour différentes positions des manches

struct CasManches {
  const char* nom;
  Pulses pulses;
};

class ManchesTest : public BaseRoulanteTest, public ::testing::WithParamInterface<CasManches> {};

TEST_P(ManchesTest, LesRouesSuiventLesConsignesCalculees) {
  piloter(GetParam().pulses);
  const Consignes attendues = consignes_attendues();
  ASSERT_GT(attendues.max_abs(), 0) << "ce cas doit produire un mouvement";
  regler_moteurs_pour(attendues);

  executer_ms(12000);

  verifier_vitesses(attendues);
  verifier_commandes_dans_les_bornes();
}

INSTANTIATE_TEST_SUITE_P(
    Manches, ManchesTest,
    ::testing::Values(CasManches{"avant", {1700, 1500, 1500}}, CasManches{"arriere", {1300, 1500, 1500}},
                      CasManches{"droite", {1500, 1700, 1500}}, CasManches{"gauche", {1500, 1300, 1500}},
                      CasManches{"rotation_horaire", {1500, 1500, 1700}},
                      CasManches{"rotation_antihoraire", {1500, 1500, 1300}},
                      CasManches{"diagonale_et_rotation", {1700, 1700, 1700}}),
    [](const ::testing::TestParamInfo<CasManches>& info) { return std::string(info.param.nom); });

// ------------------------------------------------------------------------------------------------
// Pleine échelle, changement de sens, roue libre

TEST_F(BaseRoulanteTest, PleinAvant_RouesMotricesPleinePuissanceRoueArriereArretee) {
  piloter(Pulses{PULSE_MAXIMUM, PULSE_NEUTRE, PULSE_NEUTRE});
  executer_ms(2000);

  // La consigne (255 unités) dépasse largement ce que peut atteindre le moteur simulé : la régulation
  // sature et la commande doit valoir le maximum sur les deux roues motrices.
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_DROIT), 255);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_GAUCHE), 255);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_ARRIERE), 0);
  // Directions opposées pour les deux roues motrices d'une translation avant.
  EXPECT_LT(droit.vitesse(), 0);
  EXPECT_GT(gauche.vitesse(), 0);
  verifier_commandes_dans_les_bornes();
}

TEST_F(BaseRoulanteTest, InversionDuSens_LaRegulationSuitLaNouvelleDirection) {
  piloter(Pulses{1700, PULSE_NEUTRE, PULSE_NEUTRE});
  const Consignes avant = consignes_attendues();
  regler_moteurs_pour(avant);
  executer_ms(10000);
  verifier_vitesses(avant);

  piloter(Pulses{1300, PULSE_NEUTRE, PULSE_NEUTRE});
  const Consignes arriere_c = consignes_attendues();
  // Les manches inversés doivent demander l'opposé (à 1 unité près : le mapping RC est entier).
  ASSERT_LT(arriere_c.gauche, 0);
  ASSERT_NEAR(arriere_c.gauche, -avant.gauche, 0.1 * avant.max_abs() + 1.0);
  executer_ms(15000);
  verifier_vitesses(arriere_c);
  verifier_commandes_dans_les_bornes();
}

TEST_F(BaseRoulanteTest, RelachementDesManches_LaPuissanceRetombeEtLaRoueRalentit) {
  // Propriétés vraies quelle que soit la façon de ralentir (roue libre ou freinage actif) :
  // la commande ne remonte pas après le relâchement, finit par s'annuler, et la roue ralentit.
  piloter(Pulses{1700, PULSE_NEUTRE, PULSE_NEUTRE});
  regler_moteurs_pour(consignes_attendues());
  executer_ms(10000);
  const double vitesse_initiale = gauche.vitesse();
  ASSERT_GT(vitesse_initiale, 0.5) << "le moteur doit tourner avant le relâchement";
  const int commande_initiale = pwm(PIN_VITESSE_MOTEUR_GAUCHE);

  piloter(Pulses{});
  for (int ms = 0; ms < 3000; ++ms) {
    pas();
    ASSERT_LE(pwm(PIN_VITESSE_MOTEUR_GAUCHE), commande_initiale) << "t=" << ms << " ms";
  }
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_DROIT), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_GAUCHE), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_ARRIERE), 0);
  EXPECT_LT(std::fabs(gauche.vitesse()), 0.5 * vitesse_initiale);
  verifier_commandes_dans_les_bornes();
}

TEST_F(BaseRoulanteTest, RelachementDesManches_ImpulsionDeFreinageInverseRalentitLesRoues) {
  // setup() active le freinage par impulsion inverse : au relâchement, la broche de sens bascule pendant
  // une courte impulsion de PWM, puis la commande retombe à 0 et la roue est largement ralentie, sans
  // repartir franchement à l'envers. Pleine manche pour que la vitesse soit assez grande par rapport à
  // FREINAGE_VITESSE_REFERENCE.
  piloter(Pulses{PULSE_MAXIMUM, PULSE_NEUTRE, PULSE_NEUTRE});
  regler_moteurs_pour(consignes_attendues());
  executer_ms(8000);
  const double vitesse_initiale = gauche.vitesse();
  ASSERT_GT(vitesse_initiale, 10 * FREINAGE_VITESSE_MINIMUM) << "le moteur doit tourner avant le relâchement";
  const int sens_avant = etat.numerique[PIN_SENS_MOTEUR_GAUCHE];

  piloter(Pulses{});
  pas();
  EXPECT_NE(etat.numerique[PIN_SENS_MOTEUR_GAUCHE], sens_avant) << "l'impulsion doit être de sens opposé";
  EXPECT_GT(pwm(PIN_VITESSE_MOTEUR_GAUCHE), 0);

  int duree_ms = 1;
  double vitesse_min = gauche.vitesse();
  while (pwm(PIN_VITESSE_MOTEUR_GAUCHE) > 0 && duree_ms < 1000) {
    pas();
    ++duree_ms;
    vitesse_min = std::min(vitesse_min, gauche.vitesse());
  }
  EXPECT_LE(duree_ms * 1000.0, FREINAGE_DUREE_MAX + 2000) << "durée de l'impulsion";
  EXPECT_LT(gauche.vitesse(), 0.5 * vitesse_initiale) << "la roue doit être largement ralentie";
  EXPECT_GT(vitesse_min, -0.25 * vitesse_initiale) << "la roue ne doit pas repartir franchement à l'envers";

  executer_ms(2000);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_DROIT), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_GAUCHE), 0);
  EXPECT_EQ(pwm(PIN_VITESSE_MOTEUR_ARRIERE), 0);
  verifier_commandes_dans_les_bornes();
}

}  // namespace

#include "lanceur_gtest.h"
