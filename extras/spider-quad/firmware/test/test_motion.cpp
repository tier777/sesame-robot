#include <cmath>

#include "check.h"
#include "src/core/motion.h"

using namespace spider;

TEST("gait keeps three feet down") {
  for (uint16_t cyc : {600, 1200, 2000}) {
    for (Dir d : {Dir::Forward, Dir::Backward, Dir::TurnLeft, Dir::TurnRight}) {
      GaitParams p;
      p.cycleMs = cyc;
      for (uint32_t t = 0; t < 3u * cyc; t += 5) {
        Pose q = sampleGait(d, p, t);
        int up = 0;
        for (int l = 0; l < 4; ++l) up += q.a[kneeOf(l)] > 0.5f;
        CHECK(up <= 1);
      }
    }
  }
}

TEST("forward stance pushes feet back, backward pushes forward") {
  GaitParams p;  // FL is in stance at t = 600..700 (phase 0.75..0.83)
  float f1 = sampleGait(Dir::Forward, p, 600).a[hipOf(FL)];
  float f2 = sampleGait(Dir::Forward, p, 700).a[hipOf(FL)];
  CHECK(f2 < f1);
  float b1 = sampleGait(Dir::Backward, p, 600).a[hipOf(FL)];
  float b2 = sampleGait(Dir::Backward, p, 700).a[hipOf(FL)];
  CHECK(b2 > b1);
}

TEST("turn left moves sides opposite") {  // FL and FR both in stance at 700..750
  GaitParams p;
  Pose a = sampleGait(Dir::TurnLeft, p, 700), b = sampleGait(Dir::TurnLeft, p, 750);
  CHECK((b.a[hipOf(FL)] - a.a[hipOf(FL)]) * (b.a[hipOf(FR)] - a.a[hipOf(FR)]) < 0);
}

TEST("walk stops by itself after 1 s without refresh") {
  Motion m;
  m.command(Cmd::Walk, 0);
  for (uint32_t t = 0; t <= 900; t += 10) m.update(t);
  CHECK(m.mood() == Mood::Focused);
  for (uint32_t t = 900; t <= 1500; t += 10) m.update(t);
  Pose q = m.update(1500);
  for (int j = 0; j < kJoints; ++j) CHECK_NEAR(q.a[j], 0, 0.5);
  CHECK(m.mood() == Mood::Calm);
}

TEST("walk refresh keeps walking") {
  Motion m;
  for (uint32_t t = 0; t <= 3000; t += 10) {
    if (t % 300 == 0) m.command(Cmd::Walk, t);
    m.update(t);
  }
  CHECK(m.mood() == Mood::Focused);
}

TEST("no joint jumps under command spam") {
  Motion m;
  Pose prev = m.update(0);
  const Cmd seq[] = {Cmd::Walk, Cmd::Lie, Cmd::Hello, Cmd::Walk, Cmd::Sit,
                     Cmd::Dance, Cmd::Stop, Cmd::Center, Cmd::Walk};
  uint32_t t = 0;
  float worst = 0;
  for (int k = 0; k < 40; ++k) {
    m.command(seq[k % 9], t, k % 2 ? Dir::TurnRight : Dir::Forward);
    for (int s = 0; s < 7 + (k * 13) % 50; ++s) {
      t += 10;
      Pose q = m.update(t);
      for (int j = 0; j < kJoints; ++j) worst = std::fmax(worst, std::fabs(q.a[j] - prev.a[j]));
      prev = q;
    }
  }
  CHECK(worst <= 5.0f);
}

TEST("hello ends standing, lie holds") {
  Motion m;
  m.command(Cmd::Hello, 0);
  Pose q{};
  for (uint32_t t = 0; t <= 5000; t += 10) q = m.update(t);
  CHECK_NEAR(q.a[kneeOf(FR)], 0, 0.5);
  CHECK(m.mood() == Mood::Calm);
  m.command(Cmd::Lie, 5000);
  for (uint32_t t = 5000; t <= 8000; t += 10) q = m.update(t);
  CHECK_NEAR(q.a[kneeOf(RL)], 70, 0.5);
  CHECK(m.mood() == Mood::Sleepy);
}

TEST("hello lifts the front right paw") {
  Motion m;
  m.command(Cmd::Hello, 0);
  float top = 0;
  for (uint32_t t = 0; t <= 3000; t += 10) top = std::fmax(top, m.update(t).a[kneeOf(FR)]);
  CHECK(top >= 59);
  CHECK(m.mood() == Mood::Calm);
}

TEST("center mode and cycle clamp") {
  Motion m;
  m.command(Cmd::Center, 0);
  CHECK(m.centerMode());
  m.command(Cmd::Stand, 10);
  CHECK(!m.centerMode());
  m.setCycleMs(100);
  CHECK(m.cycleMs() == 600);
  m.setCycleMs(9000);
  CHECK(m.cycleMs() == 2000);
}
