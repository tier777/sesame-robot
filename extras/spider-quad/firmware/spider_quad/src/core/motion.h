// Poses, keyframe animations, creep gait and the motion controller that blends between them.
// All angles are in leg space (see joints.h). Time is in milliseconds.
#pragma once
#include <stdint.h>

#include "joints.h"

namespace spider {

struct Pose {
  float a[kJoints];
};

Pose poseStand();
Pose poseLie();
Pose poseSit();
// Blend a -> b with smoothstep easing; t is clamped to 0..1.
Pose lerpPose(const Pose &a, const Pose &b, float t);

// A keyframe is reached `ms` after the previous one.
struct Keyframe {
  Pose pose;
  uint16_t ms;
};

enum class Dir { Forward, Backward, TurnLeft, TurnRight };

struct GaitParams {
  float hipAmp = 20;
  float kneeLift = 35;
  uint16_t cycleMs = 1200;
};

// Creep gait: legs swing one at a time (RL, FL, RR, FR), so three feet are always down.
Pose sampleGait(Dir d, const GaitParams &p, uint32_t t);
// Same, but at a gait phase in cycles (any real number, only the fraction matters).
Pose sampleGaitPhase(Dir d, const GaitParams &p, float phase);

enum class Cmd { Stand, Lie, Sit, Hello, Dance, Walk, Stop, Center };
enum class Mood { Calm, Happy, Focused, Sleepy, Worried };

constexpr uint16_t kTransitionMs = 300;
constexpr uint16_t kWalkTimeoutMs = 1000;
constexpr uint16_t kCycleMinMs = 600;
constexpr uint16_t kCycleMaxMs = 2000;

class Motion {
 public:
  Motion();
  void command(Cmd c, uint32_t nowMs, Dir d = Dir::Forward);
  Pose update(uint32_t nowMs);
  bool centerMode() const { return center_; }
  Mood mood() const;
  void setCycleMs(uint16_t ms);
  uint16_t cycleMs() const { return gait_.cycleMs; }

 private:
  enum class Mode { Hold, Animate, Walk };

  void startHold(const Pose &target, Cmd kind, uint32_t now);

  Mode mode_ = Mode::Hold;
  Cmd kind_ = Cmd::Stand;  // what the robot is doing now, for mood()
  bool center_ = false;
  Pose cur_;               // last pose returned by update()
  Pose from_;              // pose at the moment of the last command
  Pose hold_;
  uint32_t since_ = 0;     // time of the last command that changed the motion
  uint32_t lastUpdate_ = 0;
  const Keyframe *anim_ = nullptr;
  int animCount_ = 0;
  Dir dir_ = Dir::Forward;
  uint32_t lastWalkCmd_ = 0;
  float phase_ = 0;        // gait phase in cycles, advanced by update()
  GaitParams gait_;
};

}  // namespace spider
