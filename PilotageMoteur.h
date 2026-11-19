#ifndef PilotageMoteur_h
#define PilotageMoteur_H

#include <PID_v1.h>

// Nombre de pulses du codeur cumulées pour mesurer la période de rotation
#define NB_PULSES_CODEUR_MOTEURS  16   // il y avait 4
// Macro calculant la vitesse de rotation du moteur en fonction de la période mesurée sur le codeur
#define vitesse_rotation_moteur(periode) (1000000.0 / (periode * 11.0 / NB_PULSES_CODEUR_MOTEURS))
// Temps (en µs) au bout duquel on considère que la vitesse de rotation est nulle si on n'a pas reçu de pulse.
#define TIMEOUT_CODEUR_MOTEURS    10000
// Periode d'écriture sur la liaison série en mode debug (µs)
#define PERIODE_ECRITURE_LS 20000
// Freinage inverse en boucle fermée (a regler sur le robot) : quand la consigne passe à 0, la PWM de freinage
// suit la vitesse mesurée. Elle diminue donc à mesure que la roue ralentit, et s'arrête quand la roue est arrêtée.
// Vitesse (unité de get_VitesseRotation) à partir de laquelle la PWM de freinage est maximale.
#define FREINAGE_VITESSE_REFERENCE  100.0
// Commande PWM maximale de freinage (0-255).
#define FREINAGE_COMMANDE_MAX       255
// PWM de freinage par unité de vitesse mesurée.
#define FREINAGE_GAIN              (FREINAGE_COMMANDE_MAX / FREINAGE_VITESSE_REFERENCE)
// Durée maximale d'un freinage (µs) : sécurité si la mesure est incohérente.
#define FREINAGE_DUREE_MAX          300000
// Vitesse (unité de get_VitesseRotation) sous laquelle le freinage s'arrête : la roue est considérée arrêtée.
// Ce seuil ne sert qu'à arrêter le freinage, jamais à le déclencher.
#define FREINAGE_VITESSE_ARRET      2.0
// Nombre de mesures de suite sans baisse de vitesse qui signalent une inversion du sens.
#define FREINAGE_MESURES_INVERSION  3
// Hausse maximale de la commande PWM écrite par période de régulation (PID_SampleTime, 20 ms).
// Évite les coups brutaux à la reprise. 255 désactive la limitation.
#define PENTE_COMMANDE_MAX          10
// Montée de l'impulsion de freinage (PWM par ms) : 255 est atteint en environ 10 ms.
#define FREINAGE_MONTEE_PAR_MS      25.0

//--------------------------------------------------------------------------------------------------------------------
template <int pinCodeur> class PilotageMoteur {
  public:
    PilotageMoteur(int8_t pin_vitesse_moteur, int8_t pin_sens_moteur, float kp, float ki, float kd);

    // Fournit la vitesse de rotation du moteur en tours/s
    double get_VitesseRotation();
    void set_ParametresPID(float kp, float ki, float kd);
    void set_ConsigneVitesse(float consigne);
    void execute(void);

    void set_DebugMode(bool debugMode) { m_debug_mode = debugMode; }
    void set_InverseSensMoteur(bool inverseSensMoteur) { m_InverseSensMoteur = inverseSensMoteur; }
    void set_FreinageInverse(bool freinageInverse) { m_FreinageInverse = freinageInverse; }
    void demande_freinage(void) { m_DemandeFreinage = true; }

  private:
    int8_t m_PinVitesseMoteur;
    int8_t m_PinSensMoteur;
    bool m_InverseSensMoteur = false;

    bool m_FreinageInverse = false;
    bool m_DemandeFreinage = false;
    bool m_EnFreinage = false;
    bool m_SensRotation = true;   // sens de la dernière consigne non nulle
    unsigned long m_DateDebutFreinage = 0;
    unsigned long m_DureeFreinage = 0;
    double m_CommandeAppliquee = 0;   // valeur réellement écrite sur la PWM (après limitation de pente)
    unsigned long m_DateCommandeAppliquee = 0;
    double m_HistoriqueFreinage[FREINAGE_MESURES_INVERSION];  // dernières mesures pendant le freinage (la plus ancienne en 0)
    unsigned long m_DateMesureFreinage = 0;       // date de la mesure correspondante

    static int             m_Compteur_impulsions_codeur;
    static unsigned long   m_Duree_impulsion_codeur;
    static unsigned long   m_Date_derniere_impulsion_codeur;

    double m_VitesseMoteurMesuree;
    double m_ConsigneVitesseMoteur;
    double m_CommandeMoteur;

    float m_Kp;
    float m_Ki;
    float m_Kd;

    int m_PID_SampleTime = 20;

    bool m_debug_mode = false;

    PID PID_moteur;

    unsigned long m_DateEcritureLiaisonSerie = 0;

    static void routineInterruptionCodeur(void);
    void demarreFreinage(void);
    void termineFreinage(void);
    void reinitialisePID(void);
    void appliqueCommande(double commande);
};

//--------------------------------------------------------------------------------------------------------------------
template <int pinCodeur> int
PilotageMoteur<pinCodeur>::m_Compteur_impulsions_codeur;

template <int pinCodeur> unsigned long
PilotageMoteur<pinCodeur>::m_Duree_impulsion_codeur = 1000000;

template <int pinCodeur> unsigned long
PilotageMoteur<pinCodeur>::m_Date_derniere_impulsion_codeur;

template <int pinCodeur> 
PilotageMoteur<pinCodeur>::PilotageMoteur(
    int8_t pin_vitesse_moteur, 
    int8_t pin_sens_moteur, 
    float kp, 
    float ki, 
    float kd ) :
  m_Kp(kp),
  m_Ki(ki),
  m_Kd(kd),
  PID_moteur(&m_VitesseMoteurMesuree, &m_CommandeMoteur, &m_ConsigneVitesseMoteur, kp, ki, kd, DIRECT) 
{
  m_PinVitesseMoteur = pin_vitesse_moteur;
  m_PinSensMoteur = pin_sens_moteur;

  attachPinChangeInterrupt(digitalPinToPCINT(pinCodeur), routineInterruptionCodeur, RISING);
  m_Date_derniere_impulsion_codeur = micros();
  m_Compteur_impulsions_codeur = 0;
  PID_moteur.SetOutputLimits(0, 255);
  PID_moteur.SetSampleTime(m_PID_SampleTime);
  PID_moteur.SetMode(AUTOMATIC);
}
//**************************************************************
template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::set_ParametresPID(float kp, float ki, float kd) {
  m_Kp = kp;
  m_Ki = ki;
  m_Kd = kd;
  PID_moteur.SetTunings(m_Kp, m_Ki, m_Kd);
}
//****************************************************************************
template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::set_ConsigneVitesse(float consigne) {
  if (consigne != 0) {
    m_SensRotation = (consigne > 0);
    m_DemandeFreinage = false;
    if (m_EnFreinage)
      termineFreinage();
  }
  m_ConsigneVitesseMoteur = abs(consigne);
  // Avec le freinage inverse, une consigne nulle ne change pas le sens : l'impulsion de freinage doit
  // aller à l'opposé du sens de rotation.
  if (!m_FreinageInverse || (consigne != 0))
    digitalWrite(m_PinSensMoteur, (consigne > 0) != m_InverseSensMoteur);
}
//**************************************************************************
template <int pinCodeur> 
double PilotageMoteur<pinCodeur>::get_VitesseRotation() {
  // if ((micros() - m_Date_derniere_impulsion_codeur) > TIMEOUT_CODEUR_MOTEURS)
  //   return 0;
  return (1000000.0 * NB_PULSES_CODEUR_MOTEURS / (double)(m_Duree_impulsion_codeur * 11)); 
}
//***************************************************************************
template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::execute() {
  m_VitesseMoteurMesuree = get_VitesseRotation();
  if (m_DemandeFreinage) {
    m_DemandeFreinage = false;
    if (m_FreinageInverse)
      demarreFreinage();
  }
  if (m_EnFreinage) {
    // Le codeur ne donne pas le sens : on ne sait pas si la roue a changé de sens. On arrête donc le freinage :
    //  - quand la roue est proche de l'arrêt (seuil bas, fin de freinage seulement) ;
    //  - quand une nouvelle mesure n'a pas diminué : le freinage étant fort, la roue a probablement inversé ;
    //  - si la roue est arrêtée (plus d'impulsion du codeur), ou si la durée de sécurité est dépassée.
    const unsigned long maintenant = micros();
    const bool roue_arretee = (maintenant - m_Date_derniere_impulsion_codeur) > 2 * m_Duree_impulsion_codeur;
    const bool duree_depassee = (maintenant - m_DateDebutFreinage) >= m_DureeFreinage;
    const bool presque_arretee = m_VitesseMoteurMesuree < FREINAGE_VITESSE_ARRET;
    bool inversion = false;
    if (m_Date_derniere_impulsion_codeur != m_DateMesureFreinage) {
      // Nouvelle mesure depuis le cycle précédent
      // Comparaison à la mesure de FREINAGE_MESURES_INVERSION mesures plus tôt : une vitesse qui n'a pas baissé
      // pendant tout ce temps indique une inversion (la comparaison à la mesure précédente serait sensible à l'arrondi)
      inversion = (m_VitesseMoteurMesuree >= m_HistoriqueFreinage[0]);
      for (int i = 0; i < FREINAGE_MESURES_INVERSION - 1; ++i)
        m_HistoriqueFreinage[i] = m_HistoriqueFreinage[i + 1];
      m_HistoriqueFreinage[FREINAGE_MESURES_INVERSION - 1] = m_VitesseMoteurMesuree;
      m_DateMesureFreinage = m_Date_derniere_impulsion_codeur;
    }
    if (roue_arretee || duree_depassee || presque_arretee || inversion)
      termineFreinage();
  }
  if (m_EnFreinage) {
    // Freinage en boucle fermée : la PWM suit la vitesse mesurée. Elle monte doucement au départ (pas de saut),
    // puis diminue avec la vitesse (pas de coupure brutale).
    const unsigned long maintenant = micros();
    const double cible = min(FREINAGE_GAIN * m_VitesseMoteurMesuree, (double)FREINAGE_COMMANDE_MAX);
    const double montee = FREINAGE_MONTEE_PAR_MS * (maintenant - m_DateCommandeAppliquee) / 1000.0;
    m_DateCommandeAppliquee = maintenant;
    m_CommandeAppliquee = min(cible, m_CommandeAppliquee + montee);
    analogWrite(m_PinVitesseMoteur, m_CommandeAppliquee);
  }
  else {
    if (m_ConsigneVitesseMoteur == 0) {
      // Consigne nulle sans freinage : roue libre, et on repart d'un PID vierge (pas de reste d'intégrale)
      m_CommandeMoteur = 0;
      reinitialisePID();
    }
    else
      PID_moteur.Compute();
    appliqueCommande(m_CommandeMoteur);
  }
  if (m_debug_mode) {
    unsigned long date_courante = micros();
    if ((date_courante - m_DateEcritureLiaisonSerie) > PERIODE_ECRITURE_LS) {
       //Serial.print("consigne = ");
       //Serial.print(m_ConsigneVitesseMoteur);
       //Serial.print(" ; commande = ");
       //Serial.print(m_CommandeMoteur);
       //Serial.print(" ; vitesse = ");
      // Serial.println(m_VitesseMoteurMesuree);
      Serial.print(m_ConsigneVitesseMoteur);
      Serial.print(" ; ");
      Serial.print(m_CommandeMoteur);
      Serial.print(" ; ");
      Serial.println(m_VitesseMoteurMesuree);
      m_DateEcritureLiaisonSerie = date_courante;
    }
  }
}
//**********************************************************************
template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::routineInterruptionCodeur(void) {
  if (++m_Compteur_impulsions_codeur == NB_PULSES_CODEUR_MOTEURS) {
    unsigned long date_courante = micros();
    m_Duree_impulsion_codeur = date_courante - m_Date_derniere_impulsion_codeur;
    m_Date_derniere_impulsion_codeur = date_courante;
    m_Compteur_impulsions_codeur = 0;
  }
}
//**********************************************************************
template <int pinCodeur>
void PilotageMoteur<pinCodeur>::demarreFreinage(void) {
  // Roue arrêtée : plus d'impulsion du codeur depuis plus de deux périodes, la vitesse mesurée est périmée.
  if ((micros() - m_Date_derniere_impulsion_codeur) > 2 * m_Duree_impulsion_codeur)
    return;
  // Le codeur ne donne pas le sens : on suppose que le moteur tourne dans le sens de la dernière consigne.
  // Pas de seuil de vitesse : à basse vitesse la PWM de freinage est naturellement faible.
  m_DureeFreinage = FREINAGE_DUREE_MAX;
  m_DateDebutFreinage = micros();
  for (int i = 0; i < FREINAGE_MESURES_INVERSION; ++i)
    m_HistoriqueFreinage[i] = m_VitesseMoteurMesuree;
  m_DateMesureFreinage = m_Date_derniere_impulsion_codeur;
  // La PWM part de 0 (roue libre) avant de monter : l'inversion du sens ne se fait pas à pleine puissance
  m_CommandeAppliquee = 0;
  m_DateCommandeAppliquee = m_DateDebutFreinage;
  m_EnFreinage = true;
  PID_moteur.SetMode(MANUAL);
  // Impulsion dans le sens opposé à la rotation
  digitalWrite(m_PinSensMoteur, m_SensRotation == m_InverseSensMoteur);
}
//**********************************************************************
template <int pinCodeur>
void PilotageMoteur<pinCodeur>::termineFreinage(void) {
  m_EnFreinage = false;
  // La commande suivante part de 0 : elle ne reprend pas la valeur (et le sens) de l'impulsion de freinage
  m_CommandeAppliquee = 0;
  reinitialisePID();
}
//**********************************************************************
template <int pinCodeur>
void PilotageMoteur<pinCodeur>::appliqueCommande(double commande) {
  // Limitation de pente sur la HAUSSE seulement : c'est elle qui donne les coups. Une baisse de PWM
  // (coupure, relâchement) n'en donne pas : la roue roule libre.
  // La pente est exprimée par période de régulation, pour ne pas dépendre de la cadence de loop().
  const unsigned long maintenant = micros();
  const unsigned long ecoule = min(maintenant - m_DateCommandeAppliquee, (unsigned long)(m_PID_SampleTime * 1000));
  m_DateCommandeAppliquee = maintenant;
  const double hausse_max = PENTE_COMMANDE_MAX * ecoule / (double)(m_PID_SampleTime * 1000);
  if (commande > m_CommandeAppliquee)
    m_CommandeAppliquee = min(commande, m_CommandeAppliquee + hausse_max);
  else
    m_CommandeAppliquee = commande;
  analogWrite(m_PinVitesseMoteur, m_CommandeAppliquee);
}
//**********************************************************************
template <int pinCodeur>
void PilotageMoteur<pinCodeur>::reinitialisePID(void) {
  // Le passage manuel -> automatique réinitialise le PID à partir de la commande : terme intégral remis à 0
  m_CommandeMoteur = 0;
  PID_moteur.SetMode(MANUAL);
  PID_moteur.SetMode(AUTOMATIC);
}
//**********************************************************************************************
#endif // #ifndef PilotageMoteur_h
