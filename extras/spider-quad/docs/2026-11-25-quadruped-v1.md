# Паучок v1 — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Прошивка и печатная механика четвероногого робота на 8 сервах MG90S 180° через PCA9685 с любой ESP32-платой.

**Architecture:** Вся логика движения (углы, позы, анимации, походка, переходы, разбор калибровки) — чистый C++ в `src/core/` без Arduino, проверяется тестами на Mac. Тонкий слой `src/hw/` (PCA9685 по Wire, OLED, Wi‑Fi-страница, NVS) связывает ядро с железом. Корпус — один параметрический `quad.scad`, детали выбираются через `-D part=...`.

**Tech Stack:** Arduino, `arduino-cli`, ядро esp32:esp32 3.3.12, библиотеки Adafruit SSD1306 2.5.17 + Adafruit GFX (уже стоят), WebServer, Preferences. Тесты: clang++ C++17, свой мини-фреймворк как в `observer-orb/orb_firmware/test`. Механика: OpenSCAD.

**Spec:** `docs/superpowers/specs/2026-11-25-quadruped-v1-design.md`

## Global Constraints

- Сервы: MG90S 180°, импульс 500 мкс = 0°, 2500 мкс = 180°, 50 Гц.
- PCA9685: адрес 0x40, каналы — бедро/колено: FL 0/1, FR 2/3, RL 4/5, RR 6/7. OLED SSD1306 128×64 на 0x3C, та же шина.
- Плата выбирается только в `board.h`: SDA, SCL, имя и пароль точки доступа. Прошивка обязана собираться для `esp32:esp32:esp32`, `esp32:esp32:esp32c3`, `esp32:esp32:esp32s3`.
- Без OLED робот обязан работать.
- Питание — пауэрбанк 5 В; измерения заряда в прошивке нет.
- Поправка нуля ±30°, направление ±1, хранятся в NVS, namespace `quad`.
- Мягкий старт: каналы включаются по одному с паузой 150 мс.
- Печать: PETG, K1 Max, сопло 0,4, без поддержек. Размеры серв — из `robot-car/car.scad`.
- Тексты интерфейса — по-русски.

## Review Focus

1. Телефон отвалился посреди ходьбы → робот сам встаёт в стойку не позже чем через 1 с после последней команды. Тест в Task 2.
2. Кнопки жмут подряд, команда приходит посреди перехода или анимации → лапы не дёргаются: между соседними кадрами по 10 мс ни один сустав не меняется больше чем на 5°. Тест в Task 2.
3. Первая загрузка или битые данные в NVS → берутся заводские поправки, мусорные значения обрезаются. Тест в Task 3.
4. В адрес пришёл мусор (`/trim?j=12&v=abc`, `/speed?ms=-5`) → номер сустава вне 0..7 отклоняется, значения обрезаются, прошивка не падает. Обрезка — тесты в Tasks 1–2, проверка индекса — в Task 4.
5. Скорость выкручена на минимум цикла (600 мс) → в воздухе всё равно не больше одной лапы. Тест в Task 2.

---

## Файлы

```
spider-quad/
  README.md                         сборка, подключение, первые шаги
  quad.scad                         корпус и лапы, параметры наверху
  tools/export.sh                   STL всех деталей + рендеры
  stl/, renders/                    результат export.sh
  firmware/
    spider_quad/
      spider_quad.ino               setup/loop: склейка ядра и железа
      board.h                       выводы и настройки под конкретную плату
      src/core/joints.h/.cpp        суставы: углы, пределы, мкс, отсчёты PCA
      src/core/motion.h/.cpp        позы, анимации, походка, контроллер движения, настроение
      src/core/calib.h/.cpp         упаковка/разбор поправок для NVS
      src/hw/pca9685.h/.cpp         драйвер PCA9685 по Wire
      src/hw/face.h/.cpp            мордочка на SSD1306
      src/hw/web.h/.cpp, web_page.h точка доступа, страница, команды
      src/hw/store.h/.cpp           NVS через Preferences
    test/
      Makefile, check.h, main.cpp   по образцу orb_firmware/test
      test_joints.cpp, test_motion.cpp, test_calib.cpp
```

Всё в `namespace spider` (`quad` — макрос в системных заголовках ESP32).

---

### Task 1: Суставы и тестовый стенд

**Files:**
- Create: `spider-quad/firmware/test/{Makefile,check.h,main.cpp,test_joints.cpp}`
- Create: `spider-quad/firmware/spider_quad/src/core/joints.h`, `joints.cpp`

**Interfaces:**
- Produces:
  - `constexpr int kJoints = 8; enum Leg { FL = 0, FR = 1, RL = 2, RR = 3 };`
  - `constexpr int hipOf(int leg)` → `leg*2`, `constexpr int kneeOf(int leg)` → `leg*2+1`, `bool isLeftLeg(int leg)` (FL, RL)
  - `struct JointCal { int8_t trim; int8_t dir; };`
  - `JointCal defaultCal(int joint)` — trim 0; dir: левые лапы +1, правые −1 (и бедро, и колено)
  - `int8_t clampTrim(int v)` — в [−30, 30]
  - `float clampJoint(int joint, float legDeg)` — бедро [−45, 45], колено [−30, 70]
  - `float servoDeg(int joint, float legDeg, const JointCal &cal)` — `90 + dir*clampJoint(...) + trim`, обрезка в [0, 180]
  - `uint16_t degToMicros(float servoDeg)` — 0 → 500, 180 → 2500, линейно, с округлением
  - `uint16_t microsToTicks(uint16_t us, float hz)` — `round(us * 4096 * hz / 1e6)`
- Соглашение «пространства лапы»: бедро > 0 — лапа к носу робота; колено 0 — стойка, > 0 — лапа поднимается.

- [ ] **Step 1: Стенд и падающий тест.** `Makefile` как в `observer-orb/orb_firmware/test` (тулчейн Xcode при наличии, `-std=c++17 -Wall -Wextra`, `SRC = ../spider_quad/src/core/*.cpp *.cpp`, `-I../spider_quad`). `check.h` с `TEST("name")`, `CHECK(cond)`, `CHECK_NEAR(a, b, eps)`. Тесты:

```cpp
TEST("joint indices") { CHECK(hipOf(RL) == 4); CHECK(kneeOf(FR) == 3); }
TEST("micros mapping") {
  CHECK(degToMicros(0) == 500); CHECK(degToMicros(90) == 1500); CHECK(degToMicros(180) == 2500);
}
TEST("pca ticks at 50 Hz") { CHECK(microsToTicks(1500, 50) == 307); CHECK(microsToTicks(500, 50) == 102); }
TEST("servo angle uses dir, trim and limits") {
  JointCal c{5, -1};
  CHECK_NEAR(servoDeg(hipOf(FR), 20, c), 75, 1e-3);   // 90 - 20 + 5
  CHECK_NEAR(servoDeg(hipOf(FL), 80, defaultCal(hipOf(FL))), 135, 1e-3); // hip clamped to 45
  CHECK_NEAR(servoDeg(kneeOf(FL), -50, defaultCal(kneeOf(FL))), 60, 1e-3); // knee clamped to -30
  CHECK_NEAR(servoDeg(kneeOf(FL), 70, JointCal{30, 1}), 180, 1e-3);      // 190 -> 180
}
TEST("default dirs mirror sides") {
  CHECK(defaultCal(hipOf(FL)).dir == 1); CHECK(defaultCal(hipOf(RR)).dir == -1);
  CHECK(clampTrim(45) == 30); CHECK(clampTrim(-99) == -30);
}
```

- [ ] **Step 2:** `make -C spider-quad/firmware/test` → ошибка сборки (нет `joints.h`).
- [ ] **Step 3:** Реализовать `joints.h/.cpp` по Interfaces.
- [ ] **Step 4:** `make -C spider-quad/firmware/test` → `tests 5, failed 0`.
- [ ] **Step 5:** Commit `Add spider quad joint mapping with host tests`.

---

### Task 2: Позы, походка и контроллер движения

**Files:**
- Create: `spider-quad/firmware/spider_quad/src/core/motion.h`, `motion.cpp`
- Test: `spider-quad/firmware/test/test_motion.cpp`

**Interfaces:**
- Consumes: Task 1 (`kJoints`, `hipOf`, `kneeOf`, `isLeftLeg`, `clampJoint`).
- Produces:
  - `struct Pose { float a[kJoints]; };` углы в пространстве лапы
  - `Pose poseStand()` — все 0. `Pose poseLie()` — бёдра 0, колени 70. `Pose poseSit()` — передние колени 0, задние 50.
  - `Pose lerpPose(const Pose &a, const Pose &b, float t)` — с smoothstep `t*t*(3-2t)`
  - `struct Keyframe { Pose pose; uint16_t ms; };` — `ms` — время перехода к этому кадру
  - `enum class Dir { Forward, Backward, TurnLeft, TurnRight };`
  - `struct GaitParams { float hipAmp = 20; float kneeLift = 35; uint16_t cycleMs = 1200; };`
  - `Pose sampleGait(Dir d, const GaitParams &p, uint32_t t)`
  - `enum class Cmd { Stand, Lie, Sit, Hello, Dance, Walk, Stop, Center };`
  - `enum class Mood { Calm, Happy, Focused, Sleepy, Worried };`
  - `class Motion { public: void command(Cmd c, uint32_t nowMs, Dir d = Dir::Forward); Pose update(uint32_t nowMs); bool centerMode() const; Mood mood() const; void setCycleMs(uint16_t ms); uint16_t cycleMs() const; };`
  - `constexpr uint16_t kTransitionMs = 300; constexpr uint16_t kWalkTimeoutMs = 1000;`

Походка (wave/creep): фаза лапы `ph = frac(t / cycleMs + off[leg])`, сдвиги RL 0, FL 0.25, RR 0.5, FR 0.75. Перенос при `ph < 0.25`: `s = ph / 0.25`, бедро `sign * hipAmp * (2s − 1)`, колено `kneeLift * sin(π s)`. Опора: `s = (ph − 0.25) / 0.75`, бедро `sign * hipAmp * (1 − 2s)`, колено 0. `sign`: Forward +1 для всех, Backward −1, TurnLeft — левые −1 и правые +1, TurnRight — наоборот.

Анимации — статические массивы `Keyframe` в `motion.cpp`:
- Hello: сесть (400 мс) → бедро FR +30 и колено FR 60 (300) → колено FR 20 (250) → колено FR 60 (250) → колено FR 20 (250) → колено FR 60 (250) → стойка (400).
- Dance: 6 кадров: попеременно наклон влево и вправо (колени одной стороны 30) и поворот бёдер ±20; каждый кадр 300 мс, в конце стойка.

Правила `Motion`:
- Любая команда сначала плавно (`kTransitionMs`, `lerpPose`) переводит из **текущей** позы в начальную позу цели. Для Walk это `sampleGait(d, p, 0)`, для анимации — её первый кадр.
- Lie и Sit держат позу. Hello и Dance по окончании переходят в стойку. Stop = Stand.
- Walk, пришедший во время ходьбы в том же направлении, только продлевает её. Если Walk не приходил `kWalkTimeoutMs`, контроллер переходит в стойку.
- Center: `centerMode() == true`, `update` возвращает стойку. Любая другая команда выключает режим.
- `setCycleMs` обрезает в [600, 2000].
- `mood()`: стойка → Calm, ходьба → Focused, Hello/Dance → Happy, Lie → Sleepy. Worried выставляет скетч, если PCA9685 не отвечает, не ядро.

- [ ] **Step 1: Падающие тесты.**

```cpp
TEST("gait keeps three feet down") {           // Review Focus 5
  for (uint16_t cyc : {600, 1200, 2000}) for (Dir d : {Dir::Forward, Dir::TurnLeft}) {
    GaitParams p; p.cycleMs = cyc;
    for (uint32_t t = 0; t < 3u * cyc; t += 5) {
      Pose q = sampleGait(d, p, t); int up = 0;
      for (int l = 0; l < 4; ++l) up += q.a[kneeOf(l)] > 0.5f;
      CHECK(up <= 1);
    }
  }
}
TEST("forward stance pushes feet back, backward pushes forward") {
  GaitParams p;  // FL in stance at t = 0.5*cycle (phase 0.75)
  float f1 = sampleGait(Dir::Forward, p, 600).a[hipOf(FL)], f2 = sampleGait(Dir::Forward, p, 700).a[hipOf(FL)];
  CHECK(f2 < f1);
  float b1 = sampleGait(Dir::Backward, p, 600).a[hipOf(FL)], b2 = sampleGait(Dir::Backward, p, 700).a[hipOf(FL)];
  CHECK(b2 > b1);
}
TEST("turn left moves sides opposite") {   // FL and FR both in stance at 700..750
  GaitParams p;
  Pose a = sampleGait(Dir::TurnLeft, p, 700), b = sampleGait(Dir::TurnLeft, p, 750);
  CHECK((b.a[hipOf(FL)] - a.a[hipOf(FL)]) * (b.a[hipOf(FR)] - a.a[hipOf(FR)]) < 0);
}
TEST("walk stops by itself after 1 s without refresh") {   // Review Focus 1
  Motion m; m.command(Cmd::Walk, 0);
  for (uint32_t t = 0; t <= 900; t += 10) m.update(t);
  CHECK(m.mood() == Mood::Focused);
  for (uint32_t t = 900; t <= 1500; t += 10) m.update(t);
  Pose q = m.update(1500);
  for (int j = 0; j < kJoints; ++j) CHECK_NEAR(q.a[j], 0, 0.5);
}
TEST("walk refresh keeps walking") {
  Motion m; for (uint32_t t = 0; t <= 3000; t += 10) { if (t % 300 == 0) m.command(Cmd::Walk, t); m.update(t); }
  CHECK(m.mood() == Mood::Focused);
}
TEST("no joint jumps under command spam") {               // Review Focus 2
  Motion m; Pose prev = m.update(0);
  const Cmd seq[] = {Cmd::Walk, Cmd::Lie, Cmd::Hello, Cmd::Walk, Cmd::Sit, Cmd::Dance, Cmd::Stop, Cmd::Center, Cmd::Walk};
  uint32_t t = 0;
  for (int k = 0; k < 40; ++k) {
    m.command(seq[k % 9], t, k % 2 ? Dir::TurnRight : Dir::Forward);
    for (int s = 0; s < 7 + (k * 13) % 50; ++s) {
      t += 10; Pose q = m.update(t);
      for (int j = 0; j < kJoints; ++j) CHECK(std::fabs(q.a[j] - prev.a[j]) <= 5.0f);
      prev = q;
    }
  }
}
TEST("hello ends standing, lie holds") {
  Motion m; m.command(Cmd::Hello, 0); Pose q{};
  for (uint32_t t = 0; t <= 5000; t += 10) q = m.update(t);
  CHECK_NEAR(q.a[kneeOf(FR)], 0, 0.5);
  m.command(Cmd::Lie, 5000); for (uint32_t t = 5000; t <= 8000; t += 10) q = m.update(t);
  CHECK_NEAR(q.a[kneeOf(RL)], 70, 0.5); CHECK(m.mood() == Mood::Sleepy);
}
TEST("center mode and cycle clamp") {
  Motion m; m.command(Cmd::Center, 0); CHECK(m.centerMode());
  m.command(Cmd::Stand, 10); CHECK(!m.centerMode());
  m.setCycleMs(100); CHECK(m.cycleMs() == 600); m.setCycleMs(9000); CHECK(m.cycleMs() == 2000);
}
```

- [ ] **Step 2:** `make -C spider-quad/firmware/test` → ошибка сборки (нет `motion.h`).
- [ ] **Step 3:** Реализовать `motion.h/.cpp` по Interfaces и правилам выше. Контроллер хранит позу последнего `update`, это начало любого перехода.
- [ ] **Step 4:** `make -C spider-quad/firmware/test` → `failed 0`.
- [ ] **Step 5:** Commit `Add poses, creep gait and motion controller`.

---

### Task 3: Калибровка в NVS

**Files:**
- Create: `src/core/calib.h/.cpp`
- Test: `test/test_calib.cpp`

**Interfaces:**
- Consumes: Task 1 (`JointCal`, `defaultCal`, `clampTrim`, `kJoints`).
- Produces:
  - `constexpr size_t kCalBytes = 16;`
  - `void encodeCal(const JointCal in[kJoints], uint8_t out[kCalBytes])` — по 2 байта на сустав: trim как int8, dir как int8
  - `void decodeCal(const uint8_t *buf, size_t n, JointCal out[kJoints])` — `n != kCalBytes` или `buf == nullptr` → все `defaultCal`; иначе trim через `clampTrim`, dir не ±1 → `defaultCal(j).dir`

- [ ] **Step 1: Падающие тесты.**

```cpp
TEST("cal round trip") {
  JointCal in[kJoints]; for (int j = 0; j < kJoints; ++j) in[j] = {int8_t(j - 4), int8_t(j % 2 ? -1 : 1)};
  uint8_t buf[kCalBytes]; encodeCal(in, buf); JointCal out[kJoints]; decodeCal(buf, kCalBytes, out);
  for (int j = 0; j < kJoints; ++j) { CHECK(out[j].trim == in[j].trim); CHECK(out[j].dir == in[j].dir); }
}
TEST("bad cal falls back to defaults") {                    // Review Focus 3
  JointCal out[kJoints]; decodeCal(nullptr, 0, out);
  for (int j = 0; j < kJoints; ++j) { CHECK(out[j].trim == 0); CHECK(out[j].dir == defaultCal(j).dir); }
  uint8_t junk[kCalBytes]; for (auto &b : junk) b = 0x7F; decodeCal(junk, kCalBytes, out);
  CHECK(out[0].trim == 30); CHECK(out[0].dir == defaultCal(0).dir);
  decodeCal(junk, 10, out); CHECK(out[3].trim == 0);
}
```

- [ ] **Step 2:** `make -C spider-quad/firmware/test` → ошибка сборки.
- [ ] **Step 3:** Реализовать `calib` по Interfaces.
- [ ] **Step 4:** `make -C spider-quad/firmware/test` → `failed 0`.
- [ ] **Step 5:** Commit `Add calibration codec`.

---

### Task 4: Железо, страница управления и скетч

**Files:**
- Create: `spider_quad.ino`, `board.h`, `src/hw/pca9685.h/.cpp`, `src/hw/store.h/.cpp`, `src/hw/face.h/.cpp`, `src/hw/web.h/.cpp`, `src/hw/web_page.h`

**Interfaces:**
- Consumes: всё из Tasks 1–3.
- Produces (для README и следующих версий):
  - `class Pca9685 { public: bool begin(TwoWire &w, uint8_t addr = 0x40, float hz = 50); void setMicros(uint8_t ch, uint16_t us); void off(uint8_t ch); void allOff(); };` Регистры: MODE1 0x00 (сон 0x10, автоинкремент 0x20, рестарт 0x80), PRESCALE 0xFE = `round(25e6 / (4096 * hz)) − 1` (121 при 50 Гц), LED0_ON_L 0x06 + 4·ch, ALL_LED_OFF_H 0xFD бит 4. `begin` возвращает false, если адрес не отвечает.
  - `class Store { public: void load(JointCal out[kJoints]); void save(const JointCal in[kJoints]); uint16_t loadCycle(); void saveCycle(uint16_t ms); };` Preferences namespace `quad`, ключи `cal` (bytes) и `cycle` (по умолчанию 1200).
  - `class Face { public: bool begin(TwoWire &w); void draw(Mood m, bool center, uint32_t nowMs); };` 128×64, кадр не чаще 20 Гц, моргание раз в 3–5 с. Глаза: Calm — скруглённые прямоугольники, Happy — дуги ^ ^, Focused — суженные, Sleepy — линии, Worried — скошенные. В режиме Center — крупно «90°». Без экрана `begin` даёт false и `draw` ничего не делает.
  - Веб (`web.cpp`), точка доступа `QUAD_AP_SSID`/`QUAD_AP_PASS`, адрес 192.168.4.1:
    - `GET /` — страница из `web_page.h`;
    - `GET /cmd?c=stand|lie|sit|hello|dance|stop|center` и `/cmd?c=walk&d=fwd|back|left|right` → `Motion::command`;
    - `GET /speed?ms=N` → `setCycleMs` + `saveCycle`;
    - `GET /cal` → JSON `{"trim":[8],"dir":[8],"cycle":1200}`;
    - `GET /trim?j=0..7&v=-30..30`, `GET /dir?j=0..7` (переключить знак), `GET /save` → `Store::save`; `j` вне 0..7 или нечисловой → 400;
    - неизвестная команда → 400.
  - Страница: крестовина «вперёд/назад/влево/вправо» — пока кнопка зажата, `/cmd?c=walk` уходит каждые 300 мс, при отпускании `stop`; кнопки поз; ползунок скорости 600–2000; блок калибровки: 8 строк «ПЛ бедро … ЗП колено» с −/+, «перевернуть» и кнопками «Все в 90°» и «Сохранить».
- `board.h` по умолчанию — classic ESP32 DevKit: `QUAD_SDA 21`, `QUAD_SCL 22`, `QUAD_AP_SSID "spider-quad"`, `QUAD_AP_PASS "spiderquad"`. В комментариях готовые строки для ESP32-C3 SuperMini (8/9), ESP32-S3 (8/9) и Grove Cardputer ADV (SDA G2, SCL G1).
- `loop` каждые 10 мс: поза из `Motion::update`; в режиме Center на все 8 каналов идёт 1500 мкс, иначе `degToMicros(servoDeg(j, pose.a[j], cal[j]))`. В `setup` первый кадр стойки пишется по каналам с паузой 150 мс. Если PCA9685 не найден, это пишется в Serial и на экран, Wi‑Fi всё равно поднимается.

- [ ] **Step 1:** Написать `board.h`, `src/hw/*` и `spider_quad.ino` по Interfaces.
- [ ] **Step 2:** Сборка под три платы, каждая должна пройти без ошибок:

```bash
for b in esp32 esp32c3 esp32s3; do arduino-cli compile --fqbn esp32:esp32:$b spider-quad/firmware/spider_quad || break; done
```

- [ ] **Step 3:** `make -C spider-quad/firmware/test` → `failed 0` (ядро не сломано).
- [ ] **Step 4:** Открыть `web_page.h` как HTML в браузере на столе (с заглушкой `fetch`) и проверить, что кнопки шлют нужные адреса и зажатая кнопка повторяет `walk` каждые 300 мс.
- [ ] **Step 5:** Commit `Add spider quad firmware: PCA9685, face, web control`.

---

### Task 5: Корпус и лапы

**Files:**
- Create: `spider-quad/quad.scad`, `spider-quad/tools/export.sh`, `spider-quad/stl/*.stl`, `spider-quad/renders/*.png`

**Interfaces:**
- Consumes: размеры MG90S и качалки из `robot-car/car.scad` (`sv_l 22.8`, `sv_w 12.4`, `sv_shaft_off 6`, `sv_tab_span 32.1`, `sv_tab_t 2.5`, `sv_hole_pitch 27.8`, `sv_below_tab 21`, `horn_*`) — скопировать блок параметров с тем же смыслом.
- Produces: `openscad -D 'part="<name>"'` с `part` из `body_bottom`, `body_top`, `hip_bracket`, `leg`, `head`, `assembly`.

Геометрия (параметры наверху файла):
- `body_w = 96`, `body_l = 130`, стенка 2, дно 2, `body_h = 26`. Сервы бёдер вертикально, валом вверх, ось вала в `(±36, ±53)`, корпус сервы — к центру по диагонали. Фланцы лежат на полке, винты в пазы ±0,5.
- Внутри по центру место под ESP (до 55 × 30) и PCA9685 (62 × 26) с проводами; `assert` проверяет, что прямоугольник 64 × 32 не пересекается с гнёздами серв.
- `body_top`: крышка 2,4 мм на 4 винта M3 (бобышки в углах корпуса под саморез M3×8), две пары прорезей 22 × 3 под ремешки-липучки пауэрбанка (по центру, расстояние между парами 82 мм), окно под USB-кабели.
- `hip_bracket`: карман под качалку бедра сверху, вынос `coxa_len = 25` от оси бедра до оси колена, люлька под серву колена с горизонтальным валом поперёк лапы.
- `leg`: карман под качалку колена, вынос `leg_reach = 25`, спуск `leg_drop`, рассчитанный так, чтобы в стойке под дном оставалось `ground_clear = 15` мм; `assert(ground_clear >= 10)`. Кончик скруглён R4.
- `head`: коробка 34 × 22 × 16 с окном 25 × 14 под OLED 0,96″, наклон 15° вверх, крепится к переднему краю крышки двумя M2.
- Все детали лежат на столе плоской гранью, без нависаний круче 45°.

- [ ] **Step 1:** Написать `quad.scad` и `tools/export.sh` (STL для каждой детали в `stl/`, PNG сборки спереди и сверху в `renders/`).
- [ ] **Step 2:** `bash spider-quad/tools/export.sh` → без `ERROR` и `WARNING` в выводе OpenSCAD, каждый STL больше 1 КБ, все `assert` пройдены.
- [ ] **Step 3:** Посмотреть `renders/*.png`: лапы не упираются в корпус при бедре ±45°, корпус над полом в стойке.
- [ ] **Step 4:** Commit `Add spider quad printable body and legs`.

---

### Task 6: README

**Files:**
- Create: `spider-quad/README.md`

- [ ] **Step 1:** Описать: что купить (список из спеки), схему подключения (таблица каналов, I²C, питание от двух портов пауэрбанка, конденсатор), что и как печатать (таблица STL), первый запуск: «прошить → подключиться к `spider-quad` → Все в 90° → надеть качалки → поправки → Сохранить», команду прошивки и тестов, как сменить плату в `board.h`.
- [ ] **Step 2:** Проверить, что все команды из README выполняются как написано.
- [ ] **Step 3:** Commit `Document spider quad build and first run`.
