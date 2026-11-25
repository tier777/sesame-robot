#include "store.h"

#include <Preferences.h>

#include "../core/calib.h"

namespace spider {

namespace {
constexpr const char *kNs = "quad";
constexpr uint16_t kDefaultCycle = 1200;
}  // namespace

void Store::load(JointCal out[kJoints]) {
  Preferences p;
  p.begin(kNs, false);
  uint8_t buf[kCalBytes];
  size_t n = p.getBytesLength("cal");
  if (n == kCalBytes) n = p.getBytes("cal", buf, kCalBytes);
  decodeCal(n == kCalBytes ? buf : nullptr, n, out);
  p.end();
}

void Store::save(const JointCal in[kJoints]) {
  uint8_t buf[kCalBytes];
  encodeCal(in, buf);
  Preferences p;
  p.begin(kNs, false);
  p.putBytes("cal", buf, kCalBytes);
  p.end();
}

uint16_t Store::loadCycle() {
  Preferences p;
  p.begin(kNs, false);
  uint16_t ms = p.getUShort("cycle", kDefaultCycle);
  p.end();
  return ms;
}

void Store::saveCycle(uint16_t ms) {
  Preferences p;
  p.begin(kNs, false);
  p.putUShort("cycle", ms);
  p.end();
}

}  // namespace spider
