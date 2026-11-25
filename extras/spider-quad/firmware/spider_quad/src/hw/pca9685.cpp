#include "pca9685.h"

#include "../core/joints.h"

namespace spider {

namespace {
constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kMode2 = 0x01;
constexpr uint8_t kLed0OnL = 0x06;
constexpr uint8_t kAllLedOffH = 0xFD;
constexpr uint8_t kPrescale = 0xFE;
constexpr uint8_t kSleep = 0x10, kAutoInc = 0x20, kRestart = 0x80;
constexpr uint8_t kOutDrv = 0x04;  // totem-pole outputs
constexpr uint8_t kFullOff = 0x10; // bit 4 of LEDn_OFF_H
constexpr float kOscHz = 25e6f;
}  // namespace

bool Pca9685::begin(TwoWire &w, uint8_t addr, float hz) {
  w_ = &w;
  addr_ = addr;
  hz_ = hz;
  w.beginTransmission(addr);
  if (w.endTransmission() != 0) return false;
  uint8_t prescale = uint8_t(lroundf(kOscHz / (4096.0f * hz)) - 1);
  write8(kMode1, kSleep);  // prescale can only be set while asleep
  write8(kPrescale, prescale);
  write8(kMode2, kOutDrv);
  write8(kMode1, kAutoInc);
  delay(1);
  write8(kMode1, kAutoInc | kRestart);
  allOff();
  return true;
}

void Pca9685::write8(uint8_t reg, uint8_t v) {
  w_->beginTransmission(addr_);
  w_->write(reg);
  w_->write(v);
  w_->endTransmission();
}

void Pca9685::setTicks(uint8_t ch, uint16_t on, uint16_t offTicks) {
  w_->beginTransmission(addr_);
  w_->write(uint8_t(kLed0OnL + 4 * ch));
  w_->write(uint8_t(on));
  w_->write(uint8_t(on >> 8));
  w_->write(uint8_t(offTicks));
  w_->write(uint8_t(offTicks >> 8));
  w_->endTransmission();
}

void Pca9685::setMicros(uint8_t ch, uint16_t us) {
  if (w_) setTicks(ch, 0, microsToTicks(us, hz_));
}

void Pca9685::off(uint8_t ch) {
  if (w_) setTicks(ch, 0, uint16_t(kFullOff) << 8);
}

void Pca9685::allOff() {
  if (w_) write8(kAllLedOffH, kFullOff);
}

}  // namespace spider
