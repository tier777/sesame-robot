// Calibration and gait speed kept in NVS (namespace "quad"), so they survive power-off.
#pragma once
#include "../core/joints.h"

namespace spider {

class Store {
 public:
  void load(JointCal out[kJoints]);
  void save(const JointCal in[kJoints]);
  uint16_t loadCycle();
  void saveCycle(uint16_t ms);
};

}  // namespace spider
