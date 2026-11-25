#include "check.h"
#include "src/core/joints.h"

using namespace spider;

TEST("joint indices") {
  CHECK(hipOf(RL) == 4);
  CHECK(kneeOf(FR) == 3);
  CHECK(isLeftLeg(FL) && isLeftLeg(RL) && !isLeftLeg(FR) && !isLeftLeg(RR));
}

TEST("micros mapping") {
  CHECK(degToMicros(0) == 500);
  CHECK(degToMicros(90) == 1500);
  CHECK(degToMicros(180) == 2500);
}

TEST("pca ticks at 50 Hz") {
  CHECK(microsToTicks(1500, 50) == 307);
  CHECK(microsToTicks(500, 50) == 102);
}

TEST("servo angle uses dir, trim and limits") {
  JointCal c{5, -1};
  CHECK_NEAR(servoDeg(hipOf(FR), 20, c), 75, 1e-3);                         // 90 - 20 + 5
  CHECK_NEAR(servoDeg(hipOf(FL), 80, defaultCal(hipOf(FL))), 135, 1e-3);    // hip clamped to 45
  CHECK_NEAR(servoDeg(kneeOf(FL), -50, defaultCal(kneeOf(FL))), 60, 1e-3);  // knee clamped to -30
  CHECK_NEAR(servoDeg(kneeOf(FL), 70, JointCal{30, 1}), 180, 1e-3);         // 190 -> 180
}

TEST("default dirs mirror sides") {
  CHECK(defaultCal(hipOf(FL)).dir == 1);
  CHECK(defaultCal(hipOf(RR)).dir == -1);
  CHECK(defaultCal(kneeOf(FR)).dir == -1);
  CHECK(defaultCal(kneeOf(RL)).trim == 0);
  CHECK(clampTrim(45) == 30);
  CHECK(clampTrim(-99) == -30);
}
