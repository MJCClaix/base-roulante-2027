// Faux <Arduino.h> pour exécuter le firmware sur PC (tests uniquement).
//
// Il reproduit uniquement ce que le firmware et la bibliothèque PID_v1 utilisent :
//  - une horloge simulée (micros / millis) que les tests font avancer à la main ;
//  - l'état des broches (mode, niveau numérique, valeur PWM) et leur historique daté ;
//  - les routines d'interruption enregistrées, que les tests peuvent déclencher ;
//  - un faux port série.
// Un Observateur optionnel (typiquement un mock googlemock) est prévenu de chaque appel matériel.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

typedef uint8_t byte;
typedef bool boolean;

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define A0 14
#define A1 15
#define A2 16

namespace arduino_fake {

constexpr int NB_BROCHES = 64;

struct EvenementBroche {
  unsigned long date_us;
  uint8_t broche;
  int valeur;
};

// Observateur des appels matériels (tous les appels lui sont transmis après enregistrement dans Etat).
struct Observateur {
  virtual ~Observateur() = default;
  virtual void pinMode(uint8_t /*broche*/, uint8_t /*mode*/) {}
  virtual void digitalWrite(uint8_t /*broche*/, uint8_t /*valeur*/) {}
  virtual void analogWrite(uint8_t /*broche*/, int /*valeur*/) {}
  virtual void attachPinChangeInterrupt(uint8_t /*pcint*/, void (*/*routine*/)(void), uint8_t /*mode*/) {}
};

struct Etat {
  unsigned long horloge_us = 0;
  uint8_t mode[NB_BROCHES] = {};
  int numerique[NB_BROCHES] = {};
  int analogique[NB_BROCHES] = {};
  void (*routine_front_montant[NB_BROCHES])(void) = {};
  std::vector<EvenementBroche> historique_numerique;
  std::vector<EvenementBroche> historique_analogique;
  Observateur* observateur = nullptr;

  // Efface les niveaux et les historiques. L'horloge et les routines d'interruption sont conservées
  // (les objets globaux du firmware ne s'attachent qu'une fois, à leur construction).
  void reinitialiser_broches() {
    for (int i = 0; i < NB_BROCHES; ++i) {
      numerique[i] = 0;
      analogique[i] = 0;
      mode[i] = 0;
    }
    historique_numerique.clear();
    historique_analogique.clear();
  }
};

inline Etat etat;

// Simule un front montant sur la broche d'un codeur (appelle la routine d'interruption enregistrée).
inline void declencher_front_montant(uint8_t broche) {
  if (etat.routine_front_montant[broche]) etat.routine_front_montant[broche]();
}

// Faux port série : le texte n'est conservé que si 'enregistrer' est vrai (sinon coût nul).
struct SerieFake {
  bool enregistrer = false;
  std::string sortie;

  void begin(unsigned long) {}
  void print(const char* texte) { if (enregistrer) sortie += texte; }
  template <typename T> std::enable_if_t<std::is_arithmetic<T>::value> print(T valeur) {
    if (!enregistrer) return;
    std::ostringstream flux;
    flux << valeur;
    sortie += flux.str();
  }
  template <typename T> void println(T valeur) { print(valeur); print("\n"); }
  void println() { print("\n"); }
};

inline SerieFake serie;

}  // namespace arduino_fake

#define Serial arduino_fake::serie

// ------------------------------------------------------------------------------------------------
// API Arduino utilisée par le firmware

inline unsigned long micros() { return arduino_fake::etat.horloge_us; }
inline unsigned long millis() { return arduino_fake::etat.horloge_us / 1000; }

inline void pinMode(uint8_t broche, uint8_t mode) {
  arduino_fake::etat.mode[broche] = mode;
  if (arduino_fake::etat.observateur) arduino_fake::etat.observateur->pinMode(broche, mode);
}

inline void digitalWrite(uint8_t broche, uint8_t valeur) {
  auto& e = arduino_fake::etat;
  e.numerique[broche] = valeur;
  e.historique_numerique.push_back({e.horloge_us, broche, valeur});
  if (e.observateur) e.observateur->digitalWrite(broche, valeur);
}

// Comme sur Arduino, la valeur est convertie en int (le firmware passe un double).
inline void analogWrite(uint8_t broche, int valeur) {
  auto& e = arduino_fake::etat;
  e.analogique[broche] = valeur;
  e.historique_analogique.push_back({e.horloge_us, broche, valeur});
  if (e.observateur) e.observateur->analogWrite(broche, valeur);
}

// Équivalent de la fonction map() d'Arduino (arithmétique entière).
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino définit min/max/abs comme des macros ; ici de simples gabarits suffisent.
template <typename T> T max(T a, T b) { return a > b ? a : b; }
template <typename T> T min(T a, T b) { return a < b ? a : b; }
