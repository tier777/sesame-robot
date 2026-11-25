#include "calib.h"

namespace spider {

void encodeCal(const JointCal in[kJoints], uint8_t out[kCalBytes]) {
  for (int j = 0; j < kJoints; ++j) {
    out[2 * j] = uint8_t(in[j].trim);
    out[2 * j + 1] = uint8_t(in[j].dir);
  }
}

void decodeCal(const uint8_t *buf, size_t n, JointCal out[kJoints]) {
  for (int j = 0; j < kJoints; ++j) {
    out[j] = defaultCal(j);
    if (buf == nullptr || n != kCalBytes) continue;
    out[j].trim = clampTrim(int8_t(buf[2 * j]));
    int8_t dir = int8_t(buf[2 * j + 1]);
    if (dir == 1 || dir == -1) out[j].dir = dir;
  }
}

}  // namespace spider
