// Outils de simulation partagés par les suites de tests : moteur + codeur simulés, helpers d'horloge.
#pragma once

#include <Arduino.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <new>
#include <utility>

namespace simulation {

// Nombre d'impulsions de codeur par tour, tel que codé en dur dans PilotageMoteur.h
// (« rapport de réduction 11 » : macro vitesse_rotation_moteur et get_VitesseRotation).
constexpr double IMPULSIONS_PAR_TOUR = 11.0;

constexpr unsigned long PAS_US = 1000;  // pas de simulation : 1 ms

inline void avancer_horloge(unsigned long dt_us) { arduino_fake::etat.horloge_us += dt_us; }

// Moteur à courant continu + codeur simulés.
//  - la vitesse cible est proportionnelle au PWM (0..255), signée par la broche de sens ;
//  - la vitesse réelle suit la cible avec un retard du premier ordre (inertie) ;
//  - le codeur émet des fronts montants (appel de la routine d'interruption enregistrée) à raison de
//    IMPULSIONS_PAR_TOUR par tour, quelle que soit la direction ; à l'arrêt, aucune impulsion.
// La vitesse est en tours/s, dans le repère « cinématique » : positive si la roue tourne dans le sens
// d'une consigne positive. 'polarite' (+1/-1) représente le câblage : un moteur câblé à l'envers tourne
// dans le sens contraire pour le même niveau de la broche de sens.
class MoteurSimule {
 public:
  MoteurSimule(uint8_t pin_pwm, uint8_t pin_sens, uint8_t pin_codeur, int polarite,
               double vitesse_max_tr_s, double constante_temps_s = 0.15)
      : m_pin_pwm(pin_pwm), m_pin_sens(pin_sens), m_pin_codeur(pin_codeur), m_polarite(polarite),
        m_vitesse_max(vitesse_max_tr_s), m_tau(constante_temps_s) {}

  // À appeler après avoir avancé l'horloge de dt_us.
  void avancer(unsigned long dt_us) {
    const auto& e = arduino_fake::etat;
    const double dt = dt_us * 1e-6;
    const double sens = e.numerique[m_pin_sens] ? 1.0 : -1.0;
    const double cible = m_polarite * sens * (e.analogique[m_pin_pwm] / 255.0) * m_vitesse_max;
    m_vitesse += (cible - m_vitesse) * dt / m_tau;

    m_phase += std::fabs(m_vitesse) * IMPULSIONS_PAR_TOUR * dt;
    while (m_phase >= 1.0) {
      m_phase -= 1.0;
      arduino_fake::declencher_front_montant(m_pin_codeur);
    }
  }

  double vitesse() const { return m_vitesse; }
  void imposer_vitesse(double v) { m_vitesse = v; }
  void regler_vitesse_max(double vitesse_max_tr_s) { m_vitesse_max = vitesse_max_tr_s; }

 private:
  uint8_t m_pin_pwm, m_pin_sens, m_pin_codeur;
  int m_polarite;
  double m_vitesse_max, m_tau;
  double m_vitesse = 0;
  double m_phase = 0;
};

// Envoie NB impulsions régulièrement espacées pour que la vitesse mesurée par PilotageMoteur vaille
// 'tours_par_s' (la routine d'interruption mémorise la durée de NB_PULSES impulsions).
inline void imposer_vitesse_mesuree(uint8_t pin_codeur, double tours_par_s, int nb_impulsions) {
  const double periode_us = 1e6 / (tours_par_s * IMPULSIONS_PAR_TOUR);
  double reste = 0;
  for (int i = 0; i < nb_impulsions; ++i) {
    reste += periode_us;
    const unsigned long dt = static_cast<unsigned long>(reste);
    reste -= dt;
    avancer_horloge(dt);
    arduino_fake::declencher_front_montant(pin_codeur);
  }
}

// Construit un objet dans une zone mémoire remise à zéro, comme les objets globaux d'Arduino (.bss).
// PilotageMoteur n'initialise pas tous ses membres (consigne, mesure, commande) ; sur le matériel ils
// valent 0 car l'objet est global. Une variable locale contiendrait des valeurs aléatoires (voire NaN
// propagé ensuite par le PID).
template <typename T, typename... Args> T& construire_comme_global(Args&&... args) {
  alignas(T) static unsigned char zone[sizeof(T)];
  static T* instance = nullptr;
  if (instance) instance->~T();
  std::memset(zone, 0, sizeof(zone));
  instance = new (zone) T(std::forward<Args>(args)...);
  return *instance;
}

}  // namespace simulation
