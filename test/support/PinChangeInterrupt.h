// Fausse bibliothèque PinChangeInterrupt : enregistre la routine pour que les tests la déclenchent.
// Dans ce faux, le numéro de pin-change-interrupt est simplement le numéro de broche.
#pragma once

#include <Arduino.h>

#define digitalPinToPCINT(p) (p)
#define digitalPinToPinChangeInterrupt(p) (p)

inline void attachPinChangeInterrupt(const uint8_t pcintNum, void (*userFunc)(void), const uint8_t mode) {
  auto& e = arduino_fake::etat;
  e.routine_front_montant[pcintNum] = userFunc;
  if (e.observateur) e.observateur->attachPinChangeInterrupt(pcintNum, userFunc, mode);
}
