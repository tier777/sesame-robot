// Calibration blob stored in NVS: two signed bytes (trim, dir) per joint.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "joints.h"

namespace spider {

constexpr size_t kCalBytes = kJoints * 2;

void encodeCal(const JointCal in[kJoints], uint8_t out[kCalBytes]);
// Wrong size or missing data gives defaults; trims are clamped, a dir other than +-1 gets the default.
void decodeCal(const uint8_t *buf, size_t n, JointCal out[kJoints]);

}  // namespace spider
