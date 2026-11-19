// Tests unitaires de la classe PilotageMoteur<pinCodeur> : codeur, sens, PID et commande PWM.
//
// On observe ce que la boucle de régulation produit (commande écrite sur la broche PWM, vitesse
// mesurée, niveau de la broche de sens), pas seulement ce qu'on lui envoie.
//
// Ces tests vérifient des propriétés (bornes, monotonie, convergence) plutôt que des valeurs exactes,
// pour ne pas bloquer les futures évolutions de la régulation.
#include <PID_v1.h>  // inclus ici pour que PlatformIO détecte la dépendance (il ne lit pas les #include du .ino)
#include <PinChangeInterrupt.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "PilotageMoteur.h"
#include "simulation.h"

namespace {

using ::testing::_;
using ::testing::AllOf;
using ::testing::AtLeast;
using ::testing::Ge;
using ::testing::Le;
using ::testing::Ne;
using ::testing::NiceMock;
using ::testing::NotNull;

constexpr uint8_t PIN_CODEUR = 20;
constexpr uint8_t PIN_PWM = 21;
constexpr uint8_t PIN_SENS = 22;

// Mêmes gains que dans BaseMoteurHolonome.ino ; ils sont ici propres au test (le .ino peut évoluer).
constexpr float KP = 5, KI = 20, KD = 0;

using Routine = void (*)(void);

class MaterielMock : public arduino_fake::Observateur {
 public:
  MOCK_METHOD(void, pinMode, (uint8_t, uint8_t), (override));
  MOCK_METHOD(void, digitalWrite, (uint8_t, uint8_t), (override));
  MOCK_METHOD(void, analogWrite, (uint8_t, int), (override));
  MOCK_METHOD(void, attachPinChangeInterrupt, (uint8_t, Routine, uint8_t), (override));
};

using Moteur = PilotageMoteur<PIN_CODEUR>;

class PilotageMoteurTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arduino_fake::etat.reinitialiser_broches();
    arduino_fake::etat.observateur = &materiel;
  }
  void TearDown() override { arduino_fake::etat.observateur = nullptr; }

  // Crée le moteur puis impose une vitesse mesurée quasi nulle : les compteurs du codeur sont des
  // membres static, donc partagés entre tous les tests ; on repart d'un état connu.
  Moteur& creer_moteur(bool inverse = false) {
    moteur = &simulation::construire_comme_global<Moteur>(PIN_PWM, PIN_SENS, KP, KI, KD);
    moteur->set_InverseSensMoteur(inverse);
    simulation::imposer_vitesse_mesuree(PIN_CODEUR, VITESSE_A_L_ARRET, NB_PULSES_CODEUR_MOTEURS);
    return *moteur;
  }

  // Un pas de simulation de 1 ms : le temps passe, le moteur physique réagit, la régulation s'exécute.
  void pas() {
    simulation::avancer_horloge(simulation::PAS_US);
    if (moteur_physique) moteur_physique->avancer(simulation::PAS_US);
    moteur->execute();
  }

  void executer_ms(int duree_ms) {
    for (int i = 0; i < duree_ms; ++i) pas();
  }

  static int derniere_commande() { return arduino_fake::etat.analogique[PIN_PWM]; }

  static constexpr double VITESSE_A_L_ARRET = 0.05;  // tours/s

  NiceMock<MaterielMock> materiel;
  Moteur* moteur = nullptr;
  simulation::MoteurSimule* moteur_physique = nullptr;
};

// ------------------------------------------------------------------------------------------------
// Codeur

TEST_F(PilotageMoteurTest, Constructeur_AttacheLInterruptionDuCodeurSurFrontMontant) {
  EXPECT_CALL(materiel, attachPinChangeInterrupt(PIN_CODEUR, NotNull(), RISING)).Times(1);
  creer_moteur();
}

class VitesseMesureeTest : public PilotageMoteurTest, public ::testing::WithParamInterface<double> {};

TEST_P(VitesseMesureeTest, LaVitesseMesureeCorrespondALaFrequenceDesImpulsions) {
  Moteur& m = creer_moteur();
  const double vitesse = GetParam();
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, vitesse, NB_PULSES_CODEUR_MOTEURS);
  EXPECT_NEAR(m.get_VitesseRotation(), vitesse, 0.01 * vitesse);
}

INSTANTIATE_TEST_SUITE_P(Vitesses, VitesseMesureeTest, ::testing::Values(0.5, 2.0, 10.0, 50.0));

TEST_F(PilotageMoteurTest, LaVitesseMesureeNeChangeQueToutesLesNImpulsions) {
  Moteur& m = creer_moteur();
  const double avant = m.get_VitesseRotation();
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, 10.0, NB_PULSES_CODEUR_MOTEURS - 1);
  EXPECT_DOUBLE_EQ(m.get_VitesseRotation(), avant);
}

// ------------------------------------------------------------------------------------------------
// Sens de rotation

struct CasSens {
  float consigne;
  bool inverse;
  int niveau_attendu;
};

class SensTest : public PilotageMoteurTest, public ::testing::WithParamInterface<CasSens> {};

TEST_P(SensTest, LaBrocheDeSensSuitLeSigneDeLaConsigneEtLInversion) {
  const CasSens cas = GetParam();
  Moteur& m = creer_moteur(cas.inverse);
  EXPECT_CALL(materiel, digitalWrite(PIN_SENS, cas.niveau_attendu)).Times(1);
  m.set_ConsigneVitesse(cas.consigne);
  EXPECT_EQ(arduino_fake::etat.numerique[PIN_SENS], cas.niveau_attendu);
}

INSTANTIATE_TEST_SUITE_P(
    Sens, SensTest,
    ::testing::Values(CasSens{+10, false, HIGH}, CasSens{-10, false, LOW}, CasSens{+10, true, LOW},
                      CasSens{-10, true, HIGH}));

TEST_F(PilotageMoteurTest, LeSensEstChoisiAvantDeCommanderLaVitesse_SansToucherAutreChose) {
  Moteur& m = creer_moteur();
  EXPECT_CALL(materiel, digitalWrite(PIN_SENS, _)).Times(1);
  EXPECT_CALL(materiel, digitalWrite(Ne(PIN_SENS), _)).Times(0);
  EXPECT_CALL(materiel, analogWrite(_, _)).Times(0);
  m.set_ConsigneVitesse(-5);  // seule la broche de sens bouge ; la PWM n'est écrite que par execute()
}

// ------------------------------------------------------------------------------------------------
// Sorties de la boucle de régulation

TEST_F(PilotageMoteurTest, Commande_EcriteUniquementSurLaBrocheDeVitesseEtToujoursEntre0Et255) {
  Moteur& m = creer_moteur();
  EXPECT_CALL(materiel, analogWrite(PIN_PWM, AllOf(Ge(0), Le(255)))).Times(AtLeast(1));
  EXPECT_CALL(materiel, analogWrite(Ne(PIN_PWM), _)).Times(0);

  for (float consigne : {0.f, 3.f, 100.f, 1e6f, -3.f, -1e6f, 0.f}) {
    m.set_ConsigneVitesse(consigne);
    executer_ms(300);
  }
}

TEST_F(PilotageMoteurTest, ConsigneAuDessusDeLaMesure_LaCommandeAugmenteSansDiminuer) {
  Moteur& m = creer_moteur();
  m.set_ConsigneVitesse(3.0);

  int precedente = 0;
  for (int i = 0; i < 300; ++i) {
    pas();
    ASSERT_GE(derniere_commande(), precedente) << "pas " << i;
    precedente = derniere_commande();
  }
  EXPECT_GT(precedente, 0);
}

TEST_F(PilotageMoteurTest, ConsigneNulleALArret_AucuneCommande) {
  Moteur& m = creer_moteur();
  m.set_ConsigneVitesse(0);
  executer_ms(500);
  EXPECT_EQ(derniere_commande(), 0);
}

TEST_F(PilotageMoteurTest, ConsigneRamenee_A_Zero_LaCommandeNeRemontePasEtFinitParSAnnuler) {
  // On fait d'abord monter la commande (consigne > mesure), puis on annule la consigne alors que la
  // vitesse mesurée est élevée : la commande ne doit plus jamais augmenter et doit finir à 0.
  Moteur& m = creer_moteur();
  m.set_ConsigneVitesse(4.0);
  executer_ms(1000);
  const int commande_en_marche = derniere_commande();
  ASSERT_GT(commande_en_marche, 0);

  simulation::imposer_vitesse_mesuree(PIN_CODEUR, 5.0, NB_PULSES_CODEUR_MOTEURS);
  m.set_ConsigneVitesse(0);
  int precedente = commande_en_marche;
  for (int ms = 0; ms < 5000; ++ms) {
    pas();
    ASSERT_LE(derniere_commande(), precedente) << "t=" << ms << " ms";
    precedente = derniere_commande();
  }
  EXPECT_EQ(derniere_commande(), 0);
}

// ------------------------------------------------------------------------------------------------
// Boucle fermée avec un moteur et un codeur simulés

struct CasBoucleFermee {
  float consigne;
  bool inverse;  // inversion logicielle (set_InverseSensMoteur)
  int polarite;  // câblage du moteur simulé
};

class BoucleFermeeTest : public PilotageMoteurTest, public ::testing::WithParamInterface<CasBoucleFermee> {};

TEST_P(BoucleFermeeTest, LaVitesseRejointLaConsigneSansDepassementExcessifNiSortieHorsBornes) {
  const CasBoucleFermee cas = GetParam();
  Moteur& m = creer_moteur(cas.inverse);
  simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, cas.polarite, /*vitesse_max_tr_s=*/8.0);
  moteur_physique = &physique;

  m.set_ConsigneVitesse(cas.consigne);
  const double attendu = cas.consigne;  // la roue simulée est dans le repère cinématique
  double extremum = 0;
  for (int ms = 0; ms < 10000; ++ms) {
    pas();
    if (std::fabs(physique.vitesse()) > std::fabs(extremum)) extremum = physique.vitesse();
  }

  EXPECT_NEAR(physique.vitesse(), attendu, 0.1 * std::fabs(attendu)) << "vitesse finale";
  EXPECT_LT(std::fabs(extremum), 1.5 * std::fabs(attendu)) << "dépassement";
  for (const auto& e : arduino_fake::etat.historique_analogique) {
    ASSERT_GE(e.valeur, 0);
    ASSERT_LE(e.valeur, 255);
  }
}

INSTANTIATE_TEST_SUITE_P(
    Sens, BoucleFermeeTest,
    ::testing::Values(CasBoucleFermee{+3, false, +1}, CasBoucleFermee{-3, false, +1},
                      CasBoucleFermee{+3, true, -1},   // inversion logicielle compensant le câblage
                      CasBoucleFermee{-3, true, -1}));

// ------------------------------------------------------------------------------------------------
// Freinage par impulsion inverse (set_FreinageInverse)
//
// Chaque scénario dynamique est rejoué sur plusieurs variantes du moteur simulé (simulation::GRAINES) :
// un test ne doit pas réussir par chance sur une seule configuration.
// Les constantes de freinage sont à régler sur le robot : les tests vérifient des propriétés (sens, bornes,
// arrêt, pas de repartir à l'envers, pas de coup brutal) avec une marge, pas des valeurs exactes.

constexpr double VITESSE_MAX_FREINAGE = 600.0;
constexpr float CONSIGNE_FREINAGE = 200.0f;

class FreinageTest : public PilotageMoteurTest, public ::testing::WithParamInterface<CasBoucleFermee> {
 protected:
  // Met le moteur en régime établi (consigne 'consigne'), puis relâche la consigne.
  // Renvoie la vitesse (valeur absolue) 100 ms après le relâchement.
  double vitesse_apres_relachement(bool freinage, simulation::MoteurSimule& physique) {
    Moteur& m = creer_moteur();
    m.set_FreinageInverse(freinage);
    moteur_physique = &physique;
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(5000);
    m.set_ConsigneVitesse(0);
    m.demande_freinage();
    executer_ms(100);
    return std::fabs(physique.vitesse());
  }

  void nouvelle_variante() { arduino_fake::etat.reinitialiser_broches(); }
};

TEST_P(FreinageTest, ImpulsionInverseBorneePuisArretSansRepartirALEnvers) {
  const CasBoucleFermee cas = GetParam();
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    Moteur& m = creer_moteur(cas.inverse);
    m.set_FreinageInverse(true);
    simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, cas.polarite, VITESSE_MAX_FREINAGE, 0.15,
                                      graine);
    moteur_physique = &physique;

    m.set_ConsigneVitesse(cas.consigne);
    executer_ms(5000);
    const int sens_avant = arduino_fake::etat.numerique[PIN_SENS];
    const double vitesse_initiale = std::fabs(physique.vitesse());
    ASSERT_GT(vitesse_initiale, 5.0) << "le moteur doit tourner avant le relâchement";

    m.set_ConsigneVitesse(0);
    m.demande_freinage();
    pas();
    // Le freinage part de 0 et monte en quelques ms : on laisse la montée démarrer
    for (int i = 0; i < 5 && derniere_commande() == 0; ++i)
      pas();
    EXPECT_NE(arduino_fake::etat.numerique[PIN_SENS], sens_avant) << "l'impulsion doit être de sens opposé";
    EXPECT_GT(derniere_commande(), 0);

    int duree_ms = 1;
    double vitesse_min = physique.vitesse() * (cas.consigne > 0 ? 1 : -1);
    while (derniere_commande() > 0 && duree_ms < 1000) {
      pas();
      ++duree_ms;
      vitesse_min = std::min(vitesse_min, physique.vitesse() * (cas.consigne > 0 ? 1 : -1));
    }
    EXPECT_LE(duree_ms * 1000.0, FREINAGE_DUREE_MAX + 2000) << "durée de l'impulsion";
    EXPECT_LT(std::fabs(physique.vitesse()), 0.5 * vitesse_initiale) << "le moteur doit être largement ralenti";
    EXPECT_GT(vitesse_min, -0.25 * vitesse_initiale) << "le moteur ne doit pas repartir franchement à l'envers";

    // Une seule impulsion : la commande reste nulle tant que la consigne reste nulle.
    for (int ms = 0; ms < 2000; ++ms) {
      pas();
      ASSERT_EQ(derniere_commande(), 0) << "t=" << ms << " ms";
    }
    for (const auto& e : arduino_fake::etat.historique_analogique) {
      ASSERT_GE(e.valeur, 0);
      ASSERT_LE(e.valeur, 255);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Sens, FreinageTest,
    ::testing::Values(CasBoucleFermee{+CONSIGNE_FREINAGE, false, +1}, CasBoucleFermee{-CONSIGNE_FREINAGE, false, +1},
                      CasBoucleFermee{+CONSIGNE_FREINAGE, true, -1}, CasBoucleFermee{-CONSIGNE_FREINAGE, true, -1}));

TEST_F(FreinageTest, LeFreinageRalentitPlusQueLaRoueLibre) {
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    simulation::MoteurSimule physique_libre(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    const double sans_freinage = vitesse_apres_relachement(false, physique_libre);
    simulation::MoteurSimule physique_freinee(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    const double avec_freinage = vitesse_apres_relachement(true, physique_freinee);
    EXPECT_LT(avec_freinage, 0.5 * sans_freinage);
  }
}

TEST_F(FreinageTest, BasseVitesse_LaPWMDeFreinageResteNegligeable) {
  // Pas de seuil de vitesse : à basse vitesse, la PWM de freinage est proportionnelle à la vitesse, donc faible.
  Moteur& m = creer_moteur();
  m.set_FreinageInverse(true);
  m.set_ConsigneVitesse(3.0);
  executer_ms(100);  // la vitesse mesurée reste celle de creer_moteur(), très faible
  const double vitesse = m.get_VitesseRotation();

  m.set_ConsigneVitesse(0);
  m.demande_freinage();
  arduino_fake::etat.historique_analogique.clear();
  executer_ms(100);
  for (const auto& e : arduino_fake::etat.historique_analogique)
    ASSERT_LE(e.valeur, FREINAGE_GAIN * vitesse + 1) << "t=" << e.date_us << " us";
}

TEST_F(FreinageTest, NouvelleConsignePendantLImpulsion_LaRegulationReprend) {
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    Moteur& m = creer_moteur();
    m.set_FreinageInverse(true);
    simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    moteur_physique = &physique;
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(5000);
    const int sens_avant = arduino_fake::etat.numerique[PIN_SENS];

    m.set_ConsigneVitesse(0);
    m.demande_freinage();
    pas();
    ASSERT_NE(arduino_fake::etat.numerique[PIN_SENS], sens_avant) << "l'impulsion doit avoir démarré";

    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(10000);
    // La consigne est une commande PWM (0-255), pas une vitesse en tours/s : on vérifie que la régulation
    // reprend dans le bon sens, pas que la roue atteint une vitesse précise.
    EXPECT_EQ(arduino_fake::etat.numerique[PIN_SENS], sens_avant);
    EXPECT_GT(derniere_commande(), 0) << "la commande doit être repartie";
    EXPECT_GT(physique.vitesse(), 0) << "la roue doit tourner dans le sens de la consigne";
  }
}

// ------------------------------------------------------------------------------------------------
// Consigne nulle : le freinage n'a lieu que si on le demande, et une consigne nulle ne laisse aucun
// reste de commande (terme intégral du PID).

TEST_F(FreinageTest, ConsigneNulleSansDemandeDeFreinage_AucuneImpulsionNiChangementDeSens) {
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    Moteur& m = creer_moteur();
    m.set_FreinageInverse(true);
    simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    moteur_physique = &physique;
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(5000);
    const int sens_avant = arduino_fake::etat.numerique[PIN_SENS];

    m.set_ConsigneVitesse(0);
    executer_ms(200);
    EXPECT_EQ(arduino_fake::etat.numerique[PIN_SENS], sens_avant);
  }
}

TEST_F(FreinageTest, CommandeCoupee_LaCommandeTombeATutSansResteDIntegrale) {
  Moteur& m = creer_moteur();
  m.set_ConsigneVitesse(255);
  executer_ms(5000);  // moteur non simulé : la mesure reste proche de 0, le terme intégral se charge
  ASSERT_EQ(derniere_commande(), 255);

  m.set_ConsigneVitesse(0);
  pas();
  EXPECT_EQ(derniere_commande(), 0);
}

TEST_F(FreinageTest, RoueArretee_PasDeFreinageMemeSiLaDemandeArrive) {
  // La roue est à l'arrêt : plus aucune impulsion du codeur, la dernière mesure de vitesse est périmée.
  Moteur& m = creer_moteur();
  m.set_FreinageInverse(true);
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, 5.0, NB_PULSES_CODEUR_MOTEURS);
  executer_ms(3000);
  const int sens_avant = arduino_fake::etat.numerique[PIN_SENS];

  m.set_ConsigneVitesse(0);
  m.demande_freinage();
  pas();
  EXPECT_EQ(arduino_fake::etat.numerique[PIN_SENS], sens_avant);
  EXPECT_EQ(derniere_commande(), 0);
}

// ------------------------------------------------------------------------------------------------
// Pas de coup brutal : la PWM varie au plus de PENTE_COMMANDE_MAX par cycle, et l'impulsion de freinage
// s'éteint progressivement.

TEST_F(FreinageTest, LaPWMNeVariePasPlusViteQueLaPente) {
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    Moteur& m = creer_moteur();
    simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    moteur_physique = &physique;
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    arduino_fake::etat.historique_analogique.clear();
    executer_ms(3000);
    int precedente = 0;
    for (const auto& e : arduino_fake::etat.historique_analogique) {
      // Hausses limitées (une période de 20 ms donne au plus PENTE_COMMANDE_MAX). Les baisses ne le sont pas.
      if (e.valeur > precedente)
        ASSERT_LE(e.valeur - precedente, PENTE_COMMANDE_MAX) << "t=" << e.date_us << " us";
      precedente = e.valeur;
    }
  }
}

TEST_F(FreinageTest, ImpulsionDeFreinageQuiSEteintProgressivement) {
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    Moteur& m = creer_moteur();
    m.set_FreinageInverse(true);
    simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    moteur_physique = &physique;
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(5000);

    m.set_ConsigneVitesse(0);
    m.demande_freinage();
    arduino_fake::etat.historique_analogique.clear();
    executer_ms(200);

    // Commande de freinage : jamais en hausse, et sa dernière valeur non nulle est faible (pas de coupure nette)
    // L'impulsion monte d'abord (pas de saut), puis elle ne fait que décroître jusqu'à 0
    int maximum = 0;
    for (const auto& e : arduino_fake::etat.historique_analogique) maximum = std::max(maximum, e.valeur);
    ASSERT_GT(maximum, 0) << "l'impulsion doit exister";
    int precedente = maximum;
    bool apres_maximum = false;
    int derniere_non_nulle = 0;
    for (const auto& e : arduino_fake::etat.historique_analogique) {
      if (e.valeur == 0) break;
      if (e.valeur == maximum) apres_maximum = true;
      if (apres_maximum) {
        ASSERT_LE(e.valeur, precedente) << "t=" << e.date_us << " us";
        precedente = e.valeur;
      }
      derniere_non_nulle = e.valeur;
    }
    EXPECT_LE(derniere_non_nulle, 0.25 * FREINAGE_COMMANDE_MAX);
  }
}

TEST_F(FreinageTest, ReprisePasDeCoupEnFinDeFreinage) {
  for (unsigned graine : simulation::GRAINES) {
    SCOPED_TRACE("graine " + std::to_string(graine));
    nouvelle_variante();
    Moteur& m = creer_moteur();
    m.set_FreinageInverse(true);
    simulation::MoteurSimule physique(PIN_PWM, PIN_SENS, PIN_CODEUR, +1, VITESSE_MAX_FREINAGE, 0.15, graine);
    moteur_physique = &physique;
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(5000);

    m.set_ConsigneVitesse(0);
    m.demande_freinage();
    executer_ms(FREINAGE_DUREE_MAX / 1000 + 100);  // le freinage (au plus FREINAGE_DUREE_MAX) est terminé

    // Reprise : la commande part de 0 et monte par pas bornés, sans reprendre la valeur du freinage
    arduino_fake::etat.historique_analogique.clear();
    m.set_ConsigneVitesse(CONSIGNE_FREINAGE);
    executer_ms(20);
    ASSERT_FALSE(arduino_fake::etat.historique_analogique.empty());
    EXPECT_LE(arduino_fake::etat.historique_analogique.front().valeur, PENTE_COMMANDE_MAX);
  }
}

// ------------------------------------------------------------------------------------------------
// Fin du freinage : le sens ne se mesure pas, on arrête le freinage près de l'arrêt, ou quand une nouvelle
// mesure n'a pas diminué (inversion probable).

TEST_F(FreinageTest, FinDuFreinage_QuandUneNouvelleMesureNeDiminuePlus) {
  Moteur& m = creer_moteur();
  m.set_FreinageInverse(true);
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, 50.0, NB_PULSES_CODEUR_MOTEURS);
  m.set_ConsigneVitesse(0);
  m.demande_freinage();
  for (int i = 0; i < 5 && derniere_commande() == 0; ++i) pas();  // la PWM monte depuis 0
  ASSERT_GT(derniere_commande(), 0) << "le freinage doit avoir démarré";

  // Mesures suivantes à la même vitesse : la roue n'a pas ralenti, elle a probablement inversé son sens
  for (int i = 0; i < FREINAGE_MESURES_INVERSION; ++i) {
    simulation::imposer_vitesse_mesuree(PIN_CODEUR, 50.0, NB_PULSES_CODEUR_MOTEURS);
    pas();
  }
  EXPECT_EQ(derniere_commande(), 0) << "le freinage doit s'arrêter";
}

TEST_F(FreinageTest, FinDuFreinage_PresDeLArret) {
  Moteur& m = creer_moteur();
  m.set_FreinageInverse(true);
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, FREINAGE_VITESSE_ARRET / 2, NB_PULSES_CODEUR_MOTEURS);
  m.set_ConsigneVitesse(0);
  m.demande_freinage();
  pas();
  EXPECT_EQ(derniere_commande(), 0) << "sous le seuil d'arrêt, pas de freinage";
}

TEST_F(FreinageTest, FreinageQuiDiminue_LaMesureBaisseEtLeFreinageContinue) {
  Moteur& m = creer_moteur();
  m.set_FreinageInverse(true);
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, 50.0, NB_PULSES_CODEUR_MOTEURS);
  m.set_ConsigneVitesse(0);
  m.demande_freinage();
  for (int i = 0; i < 5 && derniere_commande() == 0; ++i) pas();
  simulation::imposer_vitesse_mesuree(PIN_CODEUR, 30.0, NB_PULSES_CODEUR_MOTEURS);
  pas();
  EXPECT_GT(derniere_commande(), 0) << "une mesure plus basse ne doit pas arrêter le freinage";
}

}  // namespace

#include "lanceur_gtest.h"
