#include "check.h"
#include "src/core/calib.h"

using namespace spider;

TEST("cal round trip") {
  JointCal in[kJoints];
  for (int j = 0; j < kJoints; ++j) in[j] = {int8_t(j - 4), int8_t(j % 2 ? -1 : 1)};
  uint8_t buf[kCalBytes];
  encodeCal(in, buf);
  JointCal out[kJoints];
  decodeCal(buf, kCalBytes, out);
  for (int j = 0; j < kJoints; ++j) {
    CHECK(out[j].trim == in[j].trim);
    CHECK(out[j].dir == in[j].dir);
  }
}

TEST("bad cal falls back to defaults") {
  JointCal out[kJoints];
  decodeCal(nullptr, 0, out);
  for (int j = 0; j < kJoints; ++j) {
    CHECK(out[j].trim == 0);
    CHECK(out[j].dir == defaultCal(j).dir);
  }
  uint8_t junk[kCalBytes];
  for (auto &b : junk) b = 0x7F;
  decodeCal(junk, kCalBytes, out);
  CHECK(out[0].trim == 30);
  CHECK(out[0].dir == defaultCal(0).dir);
  decodeCal(junk, 10, out);
  CHECK(out[3].trim == 0);
}
