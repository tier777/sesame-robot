#include "motion.h"

#include <math.h>

namespace spider {

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kSwing = 0.25f;  // share of the cycle a leg spends in the air
// Phase offsets in cycles, indexed by Leg (FL, FR, RL, RR). Swing order: RL, FL, RR, FR.
constexpr float kOffset[4] = {0.25f, 0.75f, 0.0f, 0.5f};

Pose make(float hip, float knee) {
  Pose p;
  for (int l = 0; l < 4; ++l) {
    p.a[hipOf(l)] = hip;
    p.a[kneeOf(l)] = knee;
  }
  return p;
}

Pose with(Pose p, int joint, float v) {
  p.a[joint] = v;
  return p;
}

Pose sitPose() {
  Pose p = make(0, 0);
  p.a[kneeOf(RL)] = 50;
  p.a[kneeOf(RR)] = 50;
  return p;
}

// Lean to one side: raise the knees of that side so the body drops there, swing hips.
Pose lean(bool left, float hip) {
  Pose p = make(hip, 0);
  for (int l = 0; l < 4; ++l)
    if (isLeftLeg(l) == left) p.a[kneeOf(l)] = 30;
  return p;
}

struct Anims {
  Keyframe hello[7];
  Keyframe dance[6];
  Anims() {
    Pose sit = sitPose();
    Pose up = with(with(sit, hipOf(FR), 30), kneeOf(FR), 60);
    Pose down = with(up, kneeOf(FR), 20);
    hello[0] = {sit, 400};
    hello[1] = {up, 300};
    hello[2] = {down, 250};
    hello[3] = {up, 250};
    hello[4] = {down, 250};
    hello[5] = {up, 250};
    hello[6] = {make(0, 0), 400};
    dance[0] = {lean(true, 20), 300};
    dance[1] = {lean(false, -20), 300};
    dance[2] = {lean(true, 20), 300};
    dance[3] = {lean(false, -20), 300};
    dance[4] = {make(0, 20), 300};
    dance[5] = {make(0, 0), 300};
  }
};

const Anims &anims() {
  static const Anims a;
  return a;
}

float hipSign(Dir d, int leg) {
  switch (d) {
    case Dir::Forward: return 1;
    case Dir::Backward: return -1;
    case Dir::TurnLeft: return isLeftLeg(leg) ? -1 : 1;
    case Dir::TurnRight: return isLeftLeg(leg) ? 1 : -1;
  }
  return 1;
}

}  // namespace

Pose poseStand() { return make(0, 0); }
Pose poseLie() { return make(0, 70); }
Pose poseSit() { return sitPose(); }

Pose lerpPose(const Pose &a, const Pose &b, float t) {
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  float e = t * t * (3 - 2 * t);
  Pose r;
  for (int j = 0; j < kJoints; ++j) r.a[j] = a.a[j] + (b.a[j] - a.a[j]) * e;
  return r;
}

Pose sampleGaitPhase(Dir d, const GaitParams &p, float phase) {
  Pose r;
  for (int l = 0; l < 4; ++l) {
    float ph = phase + kOffset[l];
    ph -= floorf(ph);
    float sign = hipSign(d, l);
    if (ph < kSwing) {
      float s = ph / kSwing;
      r.a[hipOf(l)] = sign * p.hipAmp * (2 * s - 1);
      r.a[kneeOf(l)] = p.kneeLift * sinf(kPi * s);
    } else {
      float s = (ph - kSwing) / (1 - kSwing);
      r.a[hipOf(l)] = sign * p.hipAmp * (1 - 2 * s);
      r.a[kneeOf(l)] = 0;
    }
  }
  return r;
}

Pose sampleGait(Dir d, const GaitParams &p, uint32_t t) {
  return sampleGaitPhase(d, p, float(t % p.cycleMs) / p.cycleMs);
}

Motion::Motion() : cur_(poseStand()), from_(poseStand()), hold_(poseStand()) {}

void Motion::startHold(const Pose &target, Cmd kind, uint32_t now) {
  mode_ = Mode::Hold;
  kind_ = kind;
  from_ = cur_;
  hold_ = target;
  since_ = now;
}

void Motion::command(Cmd c, uint32_t now, Dir d) {
  if (c != Cmd::Center) center_ = false;
  switch (c) {
    case Cmd::Stand:
    case Cmd::Stop: startHold(poseStand(), Cmd::Stand, now); break;
    case Cmd::Lie: startHold(poseLie(), Cmd::Lie, now); break;
    case Cmd::Sit: startHold(poseSit(), Cmd::Sit, now); break;
    case Cmd::Center:
      center_ = true;
      startHold(poseStand(), Cmd::Stand, now);
      break;
    case Cmd::Hello:
    case Cmd::Dance:
      mode_ = Mode::Animate;
      kind_ = c;
      anim_ = c == Cmd::Hello ? anims().hello : anims().dance;
      animCount_ = c == Cmd::Hello ? 7 : 6;
      from_ = cur_;
      since_ = now;
      break;
    case Cmd::Walk:
      lastWalkCmd_ = now;
      if (mode_ == Mode::Walk && dir_ == d) break;  // just a refresh
      mode_ = Mode::Walk;
      kind_ = Cmd::Walk;
      dir_ = d;
      from_ = cur_;
      since_ = now;
      phase_ = 0;
      break;
  }
}

Pose Motion::update(uint32_t now) {
  uint32_t dt = now - lastUpdate_;
  lastUpdate_ = now;
  uint32_t el = now - since_;

  if (mode_ == Mode::Walk && now - lastWalkCmd_ > kWalkTimeoutMs) {
    startHold(poseStand(), Cmd::Stand, now);
    el = 0;
  }

  switch (mode_) {
    case Mode::Hold:
      cur_ = lerpPose(from_, hold_, float(el) / kTransitionMs);
      break;
    case Mode::Walk:
      if (el < kTransitionMs) {
        cur_ = lerpPose(from_, sampleGaitPhase(dir_, gait_, 0), float(el) / kTransitionMs);
      } else {
        // Advance the phase by real time, so a speed change never makes the legs jump.
        uint32_t walked = el - kTransitionMs;
        phase_ += float(walked < dt ? walked : dt) / gait_.cycleMs;
        phase_ -= floorf(phase_);
        cur_ = sampleGaitPhase(dir_, gait_, phase_);
      }
      break;
    case Mode::Animate: {
      Pose prev = from_;
      uint32_t t = el;
      int i = 0;
      while (i < animCount_ && t >= anim_[i].ms) {
        t -= anim_[i].ms;
        prev = anim_[i].pose;
        ++i;
      }
      if (i == animCount_) {
        cur_ = anim_[animCount_ - 1].pose;
        startHold(cur_, Cmd::Stand, now);  // every animation ends standing
      } else {
        cur_ = lerpPose(prev, anim_[i].pose, float(t) / anim_[i].ms);
      }
      break;
    }
  }
  return cur_;
}

Mood Motion::mood() const {
  switch (kind_) {
    case Cmd::Walk: return Mood::Focused;
    case Cmd::Hello:
    case Cmd::Dance: return Mood::Happy;
    case Cmd::Lie: return Mood::Sleepy;
    default: return Mood::Calm;
  }
}

void Motion::setCycleMs(uint16_t ms) {
  gait_.cycleMs = ms < kCycleMinMs ? kCycleMinMs : (ms > kCycleMaxMs ? kCycleMaxMs : ms);
}

}  // namespace spider
