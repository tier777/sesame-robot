// Проигрывает одну позу прошивки и печатает «t канал угол» на каждый setServoAngle.
// Сборка: clang++ -DPOSE_FN=runWavePose -DPOSE_CMD='"wave"' -DFRAME_DELAY=100 -DWALK_CYCLES=10 -DMOTOR_DELAY=20
#include <cstdio>
#include "Arduino.h"

void setServoAngle(uint8_t channel, int angle);
void setFace(const String& faceName);
void delayWithFace(unsigned long ms);
void enterIdle();
bool pressingCheck(String cmd, int ms);

#include "movement-sequences.h"

int frameDelay = FRAME_DELAY;
int walkCycles = WALK_CYCLES;
String currentCommand;

static unsigned long t_ms = 0;
static int press_calls = 0;

void setServoAngle(uint8_t channel, int angle) {
  std::printf("%lu %d %d\n", t_ms, (int)channel, angle);
  t_ms += MOTOR_DELAY;
}
void setFace(const String&) {}
void setFaceMode(FaceAnimMode) {}
void setFaceWithMode(const String&, FaceAnimMode) {}
void delayWithFace(unsigned long ms) { t_ms += ms; }
void enterIdle() {}

// Ходьба: 2 проверки до цикла и 4 в каждом цикле. На последней проверке последнего цикла
// команда снимается, цикл while заканчивается сам и поза уходит в stand, как при отпускании кнопки.
bool pressingCheck(String cmd, int ms) {
  t_ms += ms;
  press_calls++;
  if (press_calls == 2 + 4 * WALK_CYCLES) { currentCommand = ""; return true; }
  if (currentCommand != cmd) { runStandPose(1); return false; }
  return true;
}

int main() {
  currentCommand = POSE_CMD;
  POSE_FN();
  std::printf("END %lu\n", t_ms);
  return 0;
}
