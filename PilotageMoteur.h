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
// Freinage par impulsion inverse (a regler sur le robot). La force et la durée de l'impulsion sont
// proportionnelles à la vitesse mesurée quand la consigne passe à 0, jusqu'à FREINAGE_VITESSE_REFERENCE.
// Vitesse (unité de get_VitesseRotation) en dessous de laquelle on ne freine pas.
#define FREINAGE_VITESSE_MINIMUM    1.0
// Vitesse (unité de get_VitesseRotation) à partir de laquelle l'impulsion est maximale.
#define FREINAGE_VITESSE_REFERENCE  255.0
// Commande PWM maximale de l'impulsion de freinage (0-255).
#define FREINAGE_COMMANDE_MAX       255
// Durée maximale de l'impulsion de freinage (µs).
#define FREINAGE_DUREE_MAX          50000

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

  private:
    int8_t m_PinVitesseMoteur;
    int8_t m_PinSensMoteur;
    bool m_InverseSensMoteur = false;

    bool m_FreinageInverse = false;
    bool m_DemandeFreinage = false;
    bool m_EnFreinage = false;
    bool m_SensRotation = true;   // sens de la dernière consigne non nulle
    int m_CommandeFreinage = 0;
    unsigned long m_DateDebutFreinage = 0;
    unsigned long m_DureeFreinage = 0;

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
  else if (m_ConsigneVitesseMoteur != 0)
    m_DemandeFreinage = true;
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
  if (m_EnFreinage && ((micros() - m_DateDebutFreinage) >= m_DureeFreinage))
    termineFreinage();
  if (m_EnFreinage)
    analogWrite(m_PinVitesseMoteur, m_CommandeFreinage);
  else {
    PID_moteur.Compute();
    analogWrite(m_PinVitesseMoteur, m_CommandeMoteur);
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
  // Le codeur ne donne pas le sens : on suppose que le moteur tourne dans le sens de la dernière consigne.
  if (m_VitesseMoteurMesuree < FREINAGE_VITESSE_MINIMUM)
    return;
  double ratio = min(m_VitesseMoteurMesuree / FREINAGE_VITESSE_REFERENCE, 1.0);
  m_CommandeFreinage = FREINAGE_COMMANDE_MAX * ratio;
  m_DureeFreinage = FREINAGE_DUREE_MAX * ratio;
  m_DateDebutFreinage = micros();
  m_EnFreinage = true;
  PID_moteur.SetMode(MANUAL);
  // Impulsion dans le sens opposé à la rotation
  digitalWrite(m_PinSensMoteur, m_SensRotation == m_InverseSensMoteur);
}
//**********************************************************************
template <int pinCodeur>
void PilotageMoteur<pinCodeur>::termineFreinage(void) {
  m_EnFreinage = false;
  m_CommandeMoteur = 0;
  // Le passage en automatique initialise le PID a partir de la commande (terme intégral remis à 0)
  PID_moteur.SetMode(AUTOMATIC);
}
//**********************************************************************************************
#endif // #ifndef PilotageMoteur_h
