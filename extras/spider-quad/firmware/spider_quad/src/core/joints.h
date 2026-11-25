// Joints of the spider quad: leg-space angles -> servo angle -> pulse -> PCA9685 ticks.
// Leg space: hip > 0 swings the foot toward the robot's nose; knee 0 = standing, knee > 0 lifts the foot.
#pragma once
#include <stdint.h>

namespace spider {

constexpr int kJoints = 8;
enum Leg { FL = 0, FR = 1, RL = 2, RR = 3 };

constexpr int hipOf(int leg) { return leg * 2; }
constexpr int kneeOf(int leg) { return leg * 2 + 1; }
constexpr int legOf(int joint) { return joint / 2; }
constexpr bool isKnee(int joint) { return joint % 2 == 1; }
constexpr bool isLeftLeg(int leg) { return leg == FL || leg == RL; }

// Per-joint calibration: trim in servo degrees (-30..30), dir = +1 or -1.
struct JointCal {
  int8_t trim;
  int8_t dir;
};

constexpr int kTrimLimit = 30;
constexpr float kHipMin = -45, kHipMax = 45;
constexpr float kKneeMin = -30, kKneeMax = 70;

JointCal defaultCal(int joint);
int8_t clampTrim(int v);
float clampJoint(int joint, float legDeg);
float servoDeg(int joint, float legDeg, const JointCal &cal);
uint16_t degToMicros(float servoDeg);
uint16_t microsToTicks(uint16_t us, float hz);

}  // namespace spider
