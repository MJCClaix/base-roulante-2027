// Fausse bibliothèque ServoInput : la durée de pulse est imposée par les tests.
// getPulse() et mapDeadzone() reprennent la logique de la vraie bibliothèque (ServoInputSignal).
#pragma once

#include <Arduino.h>

template <uint8_t Broche> class ServoInputPin {
 public:
  ServoInputPin(uint16_t pulse_min = 1000, uint16_t pulse_max = 2000) : m_min(pulse_min), m_max(pulse_max) {
    m_pulse = getRangeCenter();
  }

  void attach() {}
  void detach() {}

  // Spécifique au faux : impose la durée de pulse "reçue" du récepteur (µs).
  void simuler_impulsion_us(unsigned long pulse) { m_pulse = pulse; }

  // Durée de pulse ramenée dans la plage utilisateur, comme la vraie bibliothèque.
  uint16_t getPulse() const {
    if (m_pulse < m_min) return m_min;
    if (m_pulse > m_max) return m_max;
    return static_cast<uint16_t>(m_pulse);
  }

  uint16_t getRange() const { return m_max - m_min; }
  uint16_t getRangeCenter() const { return ((m_max - m_min) / 2) + m_min; }

  long mapDeadzone(long outMin, long outMax, float zonePercent) {
    if (fabs(zonePercent) >= 1.0) return (outMin + outMax) / 2;
    uint16_t zoneUs = getRange() * fabs(zonePercent);
    return mapDeadzonePulse(outMin, outMax, zoneUs);
  }

  long mapDeadzonePulse(long outMin, long outMax, uint16_t zoneUs) {
    const long outCenter = (outMin + outMax) / 2;
    if (zoneUs > getRange()) return outCenter;

    const uint16_t ctr = getRangeCenter();
    const uint16_t zoneHalf = zoneUs / 2;
    const uint16_t zoneLow = ctr - zoneHalf;
    const uint16_t zoneHigh = ctr + zoneHalf;
    const uint16_t pulse = getPulse();

    long sortie = outCenter;
    if (pulse < zoneLow) {
      sortie = ::map(pulse, m_min, zoneLow, outMin, outCenter);
    } else if (pulse > zoneHigh) {
      sortie = ::map(pulse, zoneHigh, m_max, outCenter, outMax);
    }
    return sortie;
  }

 private:
  uint16_t m_min;
  uint16_t m_max;
  unsigned long m_pulse;
};
