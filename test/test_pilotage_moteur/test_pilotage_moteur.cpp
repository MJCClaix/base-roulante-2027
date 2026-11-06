// Tests unitaires de la classe PilotageMoteur<pinCodeur> : codeur, sens, PID et commande PWM.
//
// On observe ce que la boucle de régulation produit (commande écrite sur la broche PWM, vitesse
// mesurée, niveau de la broche de sens), pas seulement ce qu'on lui envoie.
//
// Ces tests vérifient des propriétés (bornes, monotonie, convergence) plutôt que des valeurs exactes,
// pour ne pas bloquer les futures évolutions de la régulation. Les constats liés au comportement
// actuel (« ComportementActuel ») sont dans la suite de bout en bout.
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

}  // namespace

#include "lanceur_gtest.h"
