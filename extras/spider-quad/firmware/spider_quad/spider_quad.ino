// Spider quad v1: 8 MG90S 180° servos on a PCA9685, SSD1306 face, control page over Wi-Fi.
// Board pins live in board.h. Spec: docs/superpowers/specs/2026-11-25-quadruped-v1-design.md
#include <Wire.h>

#include "board.h"
#include "src/core/joints.h"
#include "src/core/motion.h"
#include "src/hw/face.h"
#include "src/hw/pca9685.h"
#include "src/hw/store.h"
#include "src/hw/web.h"

using namespace spider;

namespace {

constexpr uint32_t kFrameMs = 10;
constexpr uint32_t kSoftStartMs = 150;
constexpr uint32_t kRetryMs = 2000;

Pca9685 pca;
Face face;
Store store;
Motion motion;
JointCal cal[kJoints];
bool pcaOk = false;
uint32_t lastFrame = 0, lastRetry = 0;

void writeJoint(int j, const Pose &p) {
  uint16_t us = motion.centerMode() ? 1500 : degToMicros(servoDeg(j, p.a[j], cal[j]));
  pca.setMicros(uint8_t(j), us);
}

// Servos jump to their first position at full speed; doing it one at a time keeps the
// current spike small enough for a power bank.
void softStart() {
  Pose p = motion.update(millis());
  for (int j = 0; j < kJoints; ++j) {
    writeJoint(j, p);
    delay(kSoftStartMs);
  }
}

bool connectPca() {
  if (!pca.begin(Wire)) return false;
  softStart();
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin(QUAD_SDA, QUAD_SCL);
  Wire.setClock(400000);
  store.load(cal);
  motion.setCycleMs(store.loadCycle());
  if (!face.begin(Wire)) Serial.println("OLED не найден на 0x3C, работаем без мордочки");
  pcaOk = connectPca();
  if (!pcaOk) Serial.println("PCA9685 не найден на 0x40, проверь SDA/SCL в board.h и питание платы");
  webBegin(WebCtx{&motion, cal, &store}, QUAD_AP_SSID, QUAD_AP_PASS);
  Serial.printf("Wi-Fi %s, страница http://192.168.4.1\n", QUAD_AP_SSID);
}

void loop() {
  webLoop();
  uint32_t now = millis();
  if (!pcaOk && now - lastRetry >= kRetryMs) {
    lastRetry = now;
    pcaOk = connectPca();
    if (pcaOk) Serial.println("PCA9685 найден");
  }
  if (now - lastFrame < kFrameMs) return;
  lastFrame = now;
  Pose p = motion.update(now);
  if (pcaOk)
    for (int j = 0; j < kJoints; ++j) writeJoint(j, p);
  face.draw(pcaOk ? motion.mood() : Mood::Worried, motion.centerMode(), now);
}
