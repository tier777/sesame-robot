// Minimal PCA9685 driver: 16 PWM channels at a fixed frequency, pulses set in microseconds.
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace spider {

class Pca9685 {
 public:
  // Returns false if nothing answers at `addr`.
  bool begin(TwoWire &w, uint8_t addr = 0x40, float hz = 50);
  void setMicros(uint8_t ch, uint16_t us);
  void off(uint8_t ch);
  void allOff();

 private:
  void write8(uint8_t reg, uint8_t v);
  void setTicks(uint8_t ch, uint16_t on, uint16_t off);

  TwoWire *w_ = nullptr;
  uint8_t addr_ = 0x40;
  float hz_ = 50;
};

}  // namespace spider
