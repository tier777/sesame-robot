// Face on a 128x64 SSD1306: two eyes that blink and change with the robot's mood.
// Redraws only when something visible changes, so the servo loop is not held up by the I2C screen.
#pragma once
#include <Wire.h>

#include "../core/motion.h"

namespace spider {

class Face {
 public:
  // Returns false when no screen answers at 0x3C; draw() then does nothing.
  bool begin(TwoWire &w);
  void draw(Mood m, bool center, uint32_t nowMs);

 private:
  bool ok_ = false;
  int shown_ = -1;  // key of the frame on screen
  uint32_t nextBlink_ = 0;
};

}  // namespace spider
