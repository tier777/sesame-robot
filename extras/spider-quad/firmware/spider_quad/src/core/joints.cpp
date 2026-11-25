#include "joints.h"

#include <math.h>

namespace spider {

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

JointCal defaultCal(int joint) {
  // Servos on the right side are mirror images of the left ones.
  return JointCal{0, int8_t(isLeftLeg(legOf(joint)) ? 1 : -1)};
}

int8_t clampTrim(int v) {
  return int8_t(v < -kTrimLimit ? -kTrimLimit : (v > kTrimLimit ? kTrimLimit : v));
}

float clampJoint(int joint, float legDeg) {
  return isKnee(joint) ? clampf(legDeg, kKneeMin, kKneeMax) : clampf(legDeg, kHipMin, kHipMax);
}

float servoDeg(int joint, float legDeg, const JointCal &cal) {
  return clampf(90.0f + cal.dir * clampJoint(joint, legDeg) + cal.trim, 0.0f, 180.0f);
}

uint16_t degToMicros(float deg) {
  return uint16_t(lroundf(500.0f + clampf(deg, 0.0f, 180.0f) * (2000.0f / 180.0f)));
}

uint16_t microsToTicks(uint16_t us, float hz) {
  return uint16_t(lroundf(us * 4096.0f * hz / 1e6f));
}

}  // namespace spider
