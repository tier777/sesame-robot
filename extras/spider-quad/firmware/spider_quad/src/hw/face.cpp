#include "face.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace spider {

namespace {

constexpr uint8_t kAddr = 0x3C;
constexpr int kEyeX[2] = {38, 90};
constexpr int kEyeY = 30;
constexpr int kEyeW = 26;
constexpr uint32_t kBlinkMs = 130;

Adafruit_SSD1306 *screen = nullptr;

void eye(int i, Mood m, bool closed) {
  Adafruit_SSD1306 &d = *screen;
  int x = kEyeX[i] - kEyeW / 2;
  if (closed || m == Mood::Sleepy) {
    d.fillRect(x, kEyeY + 6, kEyeW, 4, SSD1306_WHITE);
    return;
  }
  switch (m) {
    case Mood::Happy:  // upside-down crescents: ^ ^
      d.fillCircle(kEyeX[i], kEyeY + 8, 14, SSD1306_WHITE);
      d.fillCircle(kEyeX[i], kEyeY + 16, 14, SSD1306_BLACK);
      break;
    case Mood::Focused:
      d.fillRoundRect(x, kEyeY - 2, kEyeW, 14, 5, SSD1306_WHITE);
      break;
    case Mood::Worried: {  // brows slanting down toward the outside
      d.fillRoundRect(x, kEyeY - 12, kEyeW, 30, 7, SSD1306_WHITE);
      int in = i == 0 ? x + kEyeW : x, out = i == 0 ? x : x + kEyeW;
      d.fillTriangle(out, kEyeY - 13, in, kEyeY - 13, out, kEyeY, SSD1306_BLACK);
      break;
    }
    default:
      d.fillRoundRect(x, kEyeY - 14, kEyeW, 30, 7, SSD1306_WHITE);
      break;
  }
}

}  // namespace

bool Face::begin(TwoWire &w) {
  w.beginTransmission(kAddr);
  if (w.endTransmission() != 0) return false;
  if (!screen) screen = new Adafruit_SSD1306(128, 64, &w, -1);
  ok_ = screen->begin(SSD1306_SWITCHCAPVCC, kAddr);
  if (ok_) {
    screen->cp437(true);
    screen->clearDisplay();
    screen->display();
  }
  return ok_;
}

void Face::draw(Mood m, bool center, uint32_t now) {
  if (!ok_) return;
  bool blinking = false;
  if (now >= nextBlink_) {
    blinking = now < nextBlink_ + kBlinkMs;
    if (!blinking) nextBlink_ = now + 3000 + uint32_t(random(2000));
  }
  int key = center ? 100 : int(m) * 2 + (blinking ? 1 : 0);
  if (key == shown_) return;
  shown_ = key;

  Adafruit_SSD1306 &d = *screen;
  d.clearDisplay();
  if (center) {
    d.setTextColor(SSD1306_WHITE);
    d.setTextSize(4);
    d.setCursor(28, 18);
    d.print("90");
    d.write(248);  // degree sign in cp437
  } else {
    eye(0, m, blinking);
    eye(1, m, blinking);
  }
  d.display();
}

}  // namespace spider
