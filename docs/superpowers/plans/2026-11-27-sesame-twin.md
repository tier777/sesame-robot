# Цифровой двойник Sesame — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Двойник Sesame v1: геометрия из STEP автора, позы из кода прошивки, физика MuJoCo, отчёт по сервам, калибровочный помощник и редактор поз с зеркалом на живого робота.

**Architecture:** Конвейер из независимых шагов с файлами между ними: STEP → `out/geometry.json` + сетки; прошивка → `traces/*.json`; `params.py` + геометрия → MJCF; MJCF + записи → логи → `report.md` и видео. Калибровка и редактор поз работают поверх той же модели.

**Tech Stack:** Python 3.12 (uv), MuJoCo, OCP (`cadquery-ocp`), trimesh, coacd, numpy, imageio-ffmpeg, pytest; clang++ (Xcode CLT) для записи поз.

**Spec:** `docs/superpowers/specs/2026-11-27-sesame-twin-design.md`

## Global Constraints

- Всё живёт в `hardware/v2/`: код — `hardware/v2/twin/`, тесты — `hardware/v2/twin/tests/`, окружение — `hardware/v2/.venv` (в `.gitignore`).
- Окружение: `uv venv -p 3.12 hardware/v2/.venv`, пакеты из `hardware/v2/requirements.txt`. Тесты: `hardware/v2/.venv/bin/pytest hardware/v2/twin/tests`.
- Прошивка (`firmware/`) не меняется.
- Каналы прошивки: R1=0, R2=1, L1=2, L2=3, R4=4, R3=5, L3=6, L4=7. Выводы: `servoPins` из `firmware/sesame-firmware-main.ino`.
- Углы прошивки — целые 0–180. Геометрия в `geometry.json` — мм; в MuJoCo — СИ (м, кг, рад).
- Тайминги прошивки по умолчанию: `motorCurrentDelay` 20 мс после каждого `setServoAngle`, `frameDelay` 100 мс, `walkCycles` 10.
- MG90S: ход 0–180°, предельный момент 0,2 Н·м, скорость 600°/с (60° за 0,1 с), мёртвая зона 1°, масса 13,4 г.
- Массы: PETG 1,27 г/см³, PLA 1,24 г/см³, коэффициент заполнения 0,55; DevKit 10 г, OLED 3 г, Mini560 3 г, отсек с двумя 14500 — 50 г.
- Физика: шаг 2 мс, трение лапа–пол 0,8, успокоение 0,5 с в stand перед записью.
- Ток: `I = 0,1 А + 3,3 А/(Н·м) × |момент|` на серву.
- Видео: mp4, 640×480, 30 кадров/с.
- Сообщения, отчёт и подписи — по-русски.
- Система координат двойника: X — к мордочке (OLED), Y — влево, Z — вверх; X/Y = 0 — середина четырёх осей бёдер, Z = 0 — низ нижней крышки.

## Review Focus

- Поза, которая не собирается или падает при записи (новая функция в прошивке с неизвестным помощником) — остальные позы записываются, сломанная попадает в список с причиной.
- Робот падает или «взрывается» в симуляции (NaN, наклон больше 45°) — поза помечается, конвейер идёт дальше.
- Зеркало: угол вне безопасного диапазона или порт не открылся — на робота ничего не уходит, редактор работает дальше с сообщением.
- Подстройка рядом с краем: `fw + subtrim` выходит за 0–180 — в модель идёт обрезанный угол, как в `setServoAngle`.
- В STEP не нашлась серва или деталь сустава (автор переименовал) — понятная ошибка с именем сустава, а не модель без ноги.

---

### Task 1: Окружение и запись поз из прошивки

**Files:**
- Create: `hardware/v2/requirements.txt`, `hardware/v2/.gitignore`
- Create: `hardware/v2/twin/__init__.py`
- Create: `hardware/v2/twin/firmware_trace/arduino_stub/Arduino.h`
- Create: `hardware/v2/twin/firmware_trace/harness.cpp`
- Create: `hardware/v2/twin/firmware_trace/trace.py`
- Test: `hardware/v2/twin/tests/test_trace.py`, фикстура `hardware/v2/twin/tests/fixtures/bad_firmware/movement-sequences.h`

**Interfaces:**
- Produces: `trace.POSES: dict[str, tuple[str, str]]` — имя записи → (функция C++, значение `currentCommand`):
  `rest, stand, wave, dance, swim, point, pushup, bow, cute, freaky, worm, shake, shrug, dead, crab` (команда = имя),
  `walk → ("runWalkPose","forward")`, `walk_backward → ("runWalkBackward","backward")`,
  `turn_left → ("runTurnLeft","left")`, `turn_right → ("runTurnRight","right")`.
- Produces: `trace.build_traces(firmware_dir: Path, out_dir: Path, frame_delay=100, walk_cycles=10, motor_delay=20) -> TraceResult`,
  `TraceResult(traces: dict[str, list[tuple[int,int,int]]], failed: dict[str, str])`; каждый файл `out_dir/<имя>.json` =
  `{"pose", "frame_delay", "walk_cycles", "motor_delay", "events": [[t_ms, канал, угол], ...], "duration_ms"}`.

- [ ] **Step 1: Окружение.** `requirements.txt`: `mujoco`, `numpy`, `cadquery-ocp`, `trimesh`, `coacd`, `imageio`, `imageio-ffmpeg`, `matplotlib`, `pyserial`, `pytest`. `.gitignore`: `.venv/`, `twin/meshes/`, `twin/traces/`, `twin/out/logs/`. Создать окружение и поставить пакеты.

Run: `uv venv -p 3.12 hardware/v2/.venv && uv pip install -p hardware/v2/.venv -r hardware/v2/requirements.txt && hardware/v2/.venv/bin/python -c "import mujoco, OCP, trimesh, coacd; print('ok')"`
Expected: `ok`

- [ ] **Step 2: Тесты.**

```python
def test_stand_is_exact():
    r = build_traces(FW, tmp)
    assert r.traces["stand"] == [(0,0,135),(20,1,45),(40,2,45),(60,3,135),(80,4,0),(100,5,180),(120,6,0),(140,7,180)]

def test_wave_duration():
    # сумма delayWithFace в runWavePose + 20 мс × число setServoAngle
    ev = build_traces(FW, tmp).traces["wave"]
    assert json.load(open(tmp/"wave.json"))["duration_ms"] == expected_wave_ms(FW)  # считается в тесте разбором исходника

def test_all_poses_traced():
    r = build_traces(FW, tmp)
    assert set(r.traces) | set(r.failed) == set(POSES) and not r.failed

def test_walk_cycles():
    r = build_traces(FW, tmp, walk_cycles=3)
    loop = [e for e in r.traces["walk"] if e[1] == 0 and e[2] == 170]   # R1→170 — первый шаг цикла
    assert len(loop) == 3

def test_bad_pose_does_not_stop_others():
    r = build_traces(FIXTURES/"bad_firmware", tmp)
    assert "wave" in r.failed and "stand" in r.traces
```

Фикстура `bad_firmware/movement-sequences.h` — копия настоящей, где в `runWavePose` вызывается несуществующая `runMoonwalk()`.

- [ ] **Step 3: Запустить — падают** (`build_traces` нет).

Run: `hardware/v2/.venv/bin/pytest hardware/v2/twin/tests/test_trace.py -v` → FAIL

- [ ] **Step 4: Реализация.**
  - `Arduino.h`: `String` (конструкторы от `const char*`, `int`; `==`, `!=`, `+`), `F(x)` → `x`, `Serial` с пустыми `print/println`, `uint8_t`, `constrain`.
  - `harness.cpp` подключает `movement-sequences.h` из каталога прошивки и определяет все `extern` из него. Часы `t` в мс. `setServoAngle` пишет `t, ch, a` в stdout и делает `t += motor_delay`; `delayWithFace(ms)` — `t += ms`; лица и `enterIdle` — пустые. `pressingCheck(cmd, ms)`: `t += ms`, счётчик вызовов; пока счётчик ≤ `2 + 4·walk_cycles` и `currentCommand == cmd` — `true`; иначе `currentCommand = ""`, `runStandPose(1)`, `false`. Перед позой `currentCommand` = команда из `POSES`. Какую функцию звать — макрос `-DPOSE_FN=runWavePose -DPOSE_CMD="wave"`, параметры таймингов — тоже макросами.
  - `trace.py`: для каждой позы отдельная сборка `clang++ -std=c++17 -I arduino_stub -I <firmware_dir>` и запуск; ошибка сборки или ненулевой выход → `failed[имя] = первые строки stderr`.

- [ ] **Step 5: Тесты проходят.** Run: та же команда → PASS

- [ ] **Step 6: Commit** — `git add hardware/v2/requirements.txt hardware/v2/.gitignore hardware/v2/twin` → «Record every firmware pose as a servo timeline on the host».

---

### Task 2: Геометрия из STEP

**Files:**
- Create: `hardware/v2/twin/geometry.py`
- Test: `hardware/v2/twin/tests/test_geometry.py`

**Interfaces:**
- Produces: `geometry.build(step: Path, out_dir: Path) -> dict` и файл `out/geometry.json`:
  `{"frame": {"cad_to_twin": 4×4, "origin_mm": [x,y,z]}, "links": {имя: {"parts": [имя детали], "visual": "meshes/<имя>.stl", "hulls": ["meshes/<имя>_hull<i>.stl"], "volume_cm3": float, "material": "PETG"|"servo"|"pcb"}}, "joints": {"R1".."L4": {"parent": имя звена, "child": имя звена, "anchor_mm": [x,y,z], "axis": [x,y,z]}}, "servos": {"R1".."L4": {"case_center_mm": [...], "case_size_mm": [...], "link": имя звена}}}`.
- Имена звеньев: `torso`, `femur_R1`, `femur_R2`, `femur_L1`, `femur_L2`, `foot_R3`, `foot_R4`, `foot_L3`, `foot_L4`. Цепочки: `torso →(R1) femur_R1 →(R3) foot_R3`, `R2→R4`, `L1→L3`, `L2→L4`.
- Produces: `geometry.assign_servos(servos: list[ServoShape], parts: list[Part]) -> dict[str, str]` — серва → сустав; бросает `ValueError("не найдена серва сустава R3 …")`, если суставу не досталась серва.

- [ ] **Step 1: Тесты.**

```python
def test_eight_joints_with_expected_axes():
    g = build(STEP, tmp)
    for j in ("R1","R2","L1","L2"): assert abs(g["joints"][j]["axis"][2]) > 0.99      # бедро — вертикаль
    for j in ("R3","R4","L3","L4"): assert abs(g["joints"][j]["axis"][2]) < 0.05      # лапа — горизонталь

def test_sides_and_front():
    g = build(STEP, tmp)
    a = {j: g["joints"][j]["anchor_mm"] for j in g["joints"]}
    assert a["R1"][1] < 0 and a["L1"][1] > 0 and a["R1"][0] > 0 and a["R2"][0] < 0

def test_hips_form_a_rectangle():
    a = {j: build(STEP, tmp)["joints"][j]["anchor_mm"] for j in ("R1","R2","L1","L2")}
    assert abs(a["R1"][0] - a["L1"][0]) < 0.5 and abs(a["R1"][1] + a["L1"][1]) < 0.5
    assert abs(a["R1"][0] + a["R2"][0]) < 0.5

def test_frame_matches_printing_stl():
    # рама из STEP и Internal-Frame-v121.stl после той же матрицы — габариты совпадают до 0,5 мм
    assert np.allclose(bbox(step_frame), bbox(stl_frame), atol=0.5)

def test_missing_servo_is_named():
    with pytest.raises(ValueError, match="R3"):
        assign_servos(servos_without_one, parts)
```

- [ ] **Step 2: Запустить — падают.** Run: `hardware/v2/.venv/bin/pytest hardware/v2/twin/tests/test_geometry.py -v` → FAIL

- [ ] **Step 3: Реализация.**
  - Чтение STEP через `STEPCAFControl_Reader` + XCAF (имена и положения вхождений). Каждая деталь → сетка `BRepMesh_IncrementalMesh(shape, 0.1)` → `trimesh.Trimesh` в координатах сборки.
  - Сервы — вхождения продуктов `SG90 - Micro Servo 9g - Tower Pro*`. Ось вала — цилиндрическая грань (`GeomAbs_Cylinder`) радиусом 2,0–3,0 мм внутри сервы, направление — наружу из корпуса. Корпус — ориентированная габаритная коробка тел сервы.
  - Назначение: серва, ось которой проходит через `femur-joint-<X>` и корпус которой сидит в раме, — бедро `X`; серва, корпус которой лежит в `foot-joint-<Y>`, — лапа `Y` (её корпус едет со звеном `foot_Y`, вал — к бедру). Остальные детали: `femur-joint-*` → `femur_*`, `foot-joint-*` → `foot_*`, всё прочее → `torso`.
  - `cad_to_twin`: Z_twin — вертикаль сборки (у Fusion это Y), X_twin — к `Display_OLED_0_96_128x64`, Y_twin = Z×X; начало: X/Y — середина четырёх осей бёдер, Z — низ нижней крышки (`origin_mm` — эта точка в координатах сборки).
  - Сетки звеньев: объединение деталей, упрощение до ~20 тыс. треугольников. Столкновения: `coacd.run_coacd(mesh, threshold=0.08, max_convex_hull=4)` для каждой печатной детали, выпуклая оболочка — для серв и плат.

- [ ] **Step 4: Тесты проходят.** → PASS

- [ ] **Step 5: Commit** — `geometry.py`, тест → «Read joint axes and link meshes from the author's STEP assembly».

---

### Task 3: Параметры и перевод углов

**Files:**
- Create: `hardware/v2/twin/params.py`
- Test: `hardware/v2/twin/tests/test_params.py`

**Interfaces:**
- Consumes: `out/geometry.json` (Task 2).
- Produces: константы из Global Constraints; `CHANNEL = {"R1":0,"R2":1,"L1":2,"L2":3,"R4":4,"R3":5,"L3":6,"L4":7}`; `SPLINE_TEETH = 20`;
  `HIP_FORWARD_AT = {"R1":180,"R2":180,"L1":0,"L2":0}` (угол прошивки, при котором бедро смотрит вперёд вдоль корпуса);
  `FOOT_DOWN_AT = {"R3":180,"R4":0,"L3":0,"L4":180}` (угол, при котором лапа смотрит строго вниз);
  `STAND_FW = [135, 45, 45, 135, 0, 180, 0, 180]` (по каналам, из `runStandPose`);
  `CAD_POSE_FW = 90` для всех (сборка автора = `reference-configuration.png`: бёдра вбок, лапы горизонтально);
  `joint_sign(joint) -> int` (+1/−1 относительно оси из геометрии, чтобы выполнялись две таблицы выше);
  `fw_to_rad(joint, fw: float) -> float`, `rad_to_fw(joint, q: float) -> float`;
  `effective_fw(fw: int, subtrim: int) -> int` = `constrain(fw + subtrim, 0, 180) - subtrim`;
  `parse_subtrim(ino: Path) -> list[int]`, `parse_pins(ino: Path) -> list[int]`;
  `BATTERY = {"mass_g": 50, "center_mm": [-1, 0, 74]}` в координатах двойника (центр отсека на седле из `hardware/battery-mount`).

- [ ] **Step 1: Тесты.**

```python
def test_cad_pose_is_zero():        assert all(abs(fw_to_rad(j, 90)) < 1e-9 for j in CHANNEL)
def test_roundtrip():               assert abs(rad_to_fw("L3", fw_to_rad("L3", 37)) - 37) < 1e-9
def test_effective_fw_clamps():     assert effective_fw(178, 5) == 175 and effective_fw(90, 0) == 90 and effective_fw(3, -5) == 5
def test_subtrim_and_pins_from_firmware():
    assert parse_subtrim(INO) == [0]*8 and parse_pins(INO) == [15, 2, 23, 19, 4, 16, 17, 18]
def test_hip_forward_direction():   # в собранной модели (после Task 4 — здесь через кинематику геометрии)
    assert femur_direction("R1", 180) @ [1,0,0] > 0.996 and femur_direction("L1", 0) @ [1,0,0] > 0.996
def test_foot_down_direction():
    assert foot_direction("R3", 180) @ [0,0,-1] > 0.996 and foot_direction("L3", 0) @ [0,0,-1] > 0.996
def test_cad_pose_matches_reference():   # в позе сборки бёдра поперёк корпуса, лапы горизонтально
    assert abs(femur_direction("R1", 90) @ [1,0,0]) < 0.1 and abs(foot_direction("R3", 90) @ [0,0,1]) < 0.1
```

`femur_direction(joint, fw)` и `foot_direction(joint, fw)` — функции в `params.py`: единичный вектор от оси сустава к центру масс сетки звена (лапа — к самой дальней точке), повёрнутый по `fw_to_rad` вокруг оси из геометрии.

- [ ] **Step 2: Запустить — падают.** → FAIL
- [ ] **Step 3: Реализовать `params.py`.** Если `test_cad_pose_matches_reference` не проходит — сборка автора стоит не в 90°: определить её углы по направлениям звеньев и записать в `CAD_POSE_FW` по суставам, тест поменять на «направления в позе сборки совпадают с `CAD_POSE_FW`».
- [ ] **Step 4: Тесты проходят.** → PASS
- [ ] **Step 5: Commit** — «Map firmware angles to joint angles and read subtrim and pins».

---

### Task 4: Модель MuJoCo

**Files:**
- Create: `hardware/v2/twin/model.py`
- Test: `hardware/v2/twin/tests/test_model.py`

**Interfaces:**
- Consumes: `geometry.json`, `params`.
- Produces: `model.build_mjcf(geom: dict) -> str`, `model.load() -> tuple[mujoco.MjModel, mujoco.MjData]` (собирает и кеширует `out/sesame.xml`),
  `model.set_pose(m, d, fw: list[int], on_floor: bool = True) -> None` (ставит суставы, при `on_floor` опускает корпус так, чтобы нижняя точка касалась пола),
  `model.actuator_index(joint) -> int` (порядок актуаторов = порядок каналов 0–7).

- [ ] **Step 1: Тесты.**

```python
def test_compiles_and_counts():   m,_ = load(); assert m.njnt == 9 and m.nu == 8
def test_mass():                   m,_ = load(); assert 0.23 < m.body_subtreemass[1] < 0.30
def test_ctrl_in_firmware_degrees():
    m,d = load(); set_pose(m, d, [90]*8); d.ctrl[:] = 90; mujoco.mj_forward(m, d)
    assert np.allclose(d.actuator_force, 0, atol=1e-6)
def test_no_self_contact_at_stand():
    m,d = load(); set_pose(m, d, STAND_FW); mujoco.mj_forward(m, d)
    assert all(is_floor_pair(m, c) for c in d.contact[:d.ncon])
```

- [ ] **Step 2: Запустить — падают.** → FAIL
- [ ] **Step 3: Реализация.**
  - Тела: `torso` со `freejoint`; дети по цепочкам из Task 2. Все тела в позе сборки без поворота, `pos` = якорь сустава; сетки лежат в координатах двойника, поэтому у геомов `pos = −мировое положение тела`.
  - Шарнир: `axis` из геометрии × `joint_sign`, `range` = `fw_to_rad` от 0 и 180, `damping 0.01`, `armature 0.0005`.
  - Привод — `general`, управление прямо в градусах прошивки: `gainprm[0] = kp·s·π/180`, `biasprm = [−kp·s·π/2, −kp, −kv]`, `ctrlrange 0 180`, `forcerange −0.2 0.2`, где `kp = 2.3`, `kv = 0.02`, `s = joint_sign`.
  - Массы: печатные детали — геомы с `density = 1270 × 0.55`; сервы — коробка по `case_size_mm`, масса 13,4 г; DevKit, OLED, Mini560 — коробки на местах платы из STEP; отсек — коробка 58×31×16 мм, 50 г, в `BATTERY.center_mm`.
  - Пол — `plane`, `friction 0.8 0.005 0.0001`. Видимые сетки — без столкновений (`contype=0`), столкновения — по выпуклым частям.
- [ ] **Step 4: Тесты проходят.** → PASS
- [ ] **Step 5: Commit** — «Build a MuJoCo model of Sesame from the STEP geometry».

---

### Task 5: Симуляция поз

**Files:**
- Create: `hardware/v2/twin/simulate.py`
- Test: `hardware/v2/twin/tests/test_simulate.py`

**Interfaces:**
- Consumes: `model.load/set_pose`, `traces/*.json`, `params.effective_fw`, `params.parse_subtrim`.
- Produces: `simulate.run(trace: dict, settle_s=0.5, dt=0.002, tail_s=0.5) -> Log`; `Log` — `numpy`-массивы по шагам: `t`, `target_fw[8]`, `q_fw[8]`, `torque[8]`, `com[3]`, `torso_pos[3]`, `torso_tilt_deg`, `foot_contacts` (список точек на шаг), `contacts` (список `(тело_a, тело_b, сила)` на шаг), `status: "ok"|"fell"|"unstable"`; `simulate.save(log, path)`, `simulate.load(path) -> Log`.

- [ ] **Step 1: Тесты.**

```python
def test_stand_holds():
    log = run({"events": [], "duration_ms": 5000})          # только stand
    assert log.status == "ok" and log.torso_tilt_deg.max() < 2 and foot_slip_mm(log) < 1
def test_targets_follow_trace():
    log = run(trace("stand"));  assert np.allclose(log.target_fw[-1], STAND_FW)
def test_speed_limit():
    log = run({"events": [[0, 5, 0]], "duration_ms": 1000})
    assert np.abs(np.diff(log.target_fw[:,5]) / np.diff(log.t)).max() <= 600 + 1e-6
def test_absurd_pose_is_flagged_not_crashing():
    log = run({"events": [[0,c,0] for c in range(8)] + [[20,c,180] for c in range(8)], "duration_ms": 2000})
    assert log.status in ("ok","fell","unstable") and np.isfinite(log.com).all()
```

- [ ] **Step 2: Запустить — падают.** → FAIL
- [ ] **Step 3: Реализация.** Старт: `set_pose(STAND_FW, on_floor=True)`, `settle_s` с целями stand. Дальше события записи применяются по времени; цель каждой сервы `effective_fw(угол, subtrim)` идёт к новому значению не быстрее 600°/с и попадает в `d.ctrl`. Мёртвая зона 1°: при |цель − факт| < 1° цель не обновляется. После `duration_ms` — ещё `tail_s`. NaN в `qpos` или наклон корпуса > 45° → остановка, `status = "unstable"` / `"fell"`.
- [ ] **Step 4: Тесты проходят.** → PASS
- [ ] **Step 5: Commit** — «Play firmware poses in MuJoCo with MG90S limits».

---

### Task 6: Отчёт и видео

**Files:**
- Create: `hardware/v2/twin/report.py`
- Test: `hardware/v2/twin/tests/test_report.py`

**Interfaces:**
- Consumes: `Log` (Task 5), `model.load`.
- Produces: `report.analyze(log: Log) -> PoseStats` (по каждой серве: `fw_min, fw_max, speed_peak_dps, torque_peak_pct, torque_mean_pct, stall_s`; позы: `collisions: list[(тело_a, тело_b, t, fw[8])]`, `margin_min_mm`, `status`, `walk_dx_mm_per_cycle`, `current_peak_a`, `current_mean_a`);
  `report.current_a(torque_nm: np.ndarray) -> np.ndarray` = `0.1 + 3.3 * |τ|`;
  `report.support_margin_mm(com_xy, contacts_xy) -> float` (≥3 точки — расстояние до края выпуклой оболочки, внутри > 0; меньше 3 — минус расстояние до отрезка/точки);
  `report.write(stats: dict[str, PoseStats], failed: dict[str,str], out: Path) -> Path` → `out/report.md`;
  `report.render_video(log: Log, path: Path) -> None`.

- [ ] **Step 1: Тесты.**

```python
def test_current_model():          assert np.allclose(current_a(np.array([0.0, 0.1, 0.2])), [0.1, 0.43, 0.76])
def test_margin_inside_square():   assert support_margin_mm([0,0], [[-10,-10],[10,-10],[10,10],[-10,10]]) == pytest.approx(10)
def test_margin_two_feet():        assert support_margin_mm([0,5], [[-10,0],[10,0]]) == pytest.approx(-5)
def test_stall_detected():          # синтетический лог: момент 0,2 Н·м 0,3 с у канала 3
    assert analyze(fake_log(stall_channel=3, stall_s=0.3)).servos[3].stall_s == pytest.approx(0.3, abs=0.01)
def test_report_lists_failed():     assert "wave" in write({}, {"wave": "runMoonwalk"}, tmp).read_text()
```

- [ ] **Step 2: Запустить — падают.** → FAIL
- [ ] **Step 3: Реализация.** Столкновения — все контакты, кроме «лапа–пол»; одинаковые пары подряд схлопываются в одно событие с первым моментом. Упор — |момент| ≥ 0,2 Н·м дольше 0,2 с подряд. Видео — `mujoco.Renderer(m, 480, 640)`, камера сбоку-сверху, следит за корпусом, `imageio` → mp4. `report.md`: общая таблица поз (статус, запас устойчивости, пик и среднее тока), таблица по каждой серве (мин/макс угол во всех позах, пиковая скорость, пиковый момент, в какой позе), столкновения, ссылки на видео, сломанные позы с причиной.
- [ ] **Step 4: Тесты проходят.** → PASS
- [ ] **Step 5: Commit** — «Report servo ranges, loads, contacts and stability per pose».

---

### Task 7: Калибровочный помощник

**Files:**
- Create: `hardware/v2/twin/calibrate.py`
- Test: `hardware/v2/twin/tests/test_calibrate.py`

**Interfaces:**
- Consumes: `model.load/set_pose`, `params`.
- Produces: `calibrate.safe_range(joint: str, step_deg=1) -> tuple[int, int]` (корпус висит в воздухе, остальные суставы в 90; от 90 в обе стороны до первого контакта звена или его потомка с чем угодно, кроме своего родителя);
  `calibrate.horn_fix(computed_stop: int, measured_stop: int, teeth: int = SPLINE_TEETH) -> HornFix(teeth: int, subtrim: int)`, где `o = measured − computed`, `teeth = round(o / (360/teeth))`, `subtrim = o − teeth·360/teeth` (округлено до целого);
  `calibrate.channel_table(ino: Path) -> list[tuple[int, str, int]]` (канал, сустав, GPIO);
  CLI: `python -m twin.calibrate` — пишет `out/calibration.md` (таблица каналов, безопасные диапазоны) и картинки `out/calib/<сустав>_<0|90|180>.png`;
  `python -m twin.calibrate R1 --stop 155` — печатает «переодень качалку R1 на N зубцов в сторону меньших/больших углов, servoSubtrim[0] = S».

- [ ] **Step 1: Тесты.**

```python
def test_horn_fix():
    assert horn_fix(170, 170) == HornFix(0, 0)
    assert horn_fix(170, 155) == HornFix(-1, 3)
    assert horn_fix(90, 134) == HornFix(2, 8)
def test_channel_table():           assert channel_table(INO)[0] == (0, "R1", 15) and channel_table(INO)[5] == (5, "R3", 16)
def test_safe_range_inside_servo(): lo, hi = safe_range("R1"); assert 0 <= lo < 90 < hi <= 180
def test_pictures_written():        main([]); assert (OUT/"calib"/"R1_90.png").exists()
```

- [ ] **Step 2: Запустить — падают.** → FAIL
- [ ] **Step 3: Реализация** по интерфейсам. Картинки: нога целиком в кадре, корпус полупрозрачный, подпись угла.
- [ ] **Step 4: Тесты проходят.** → PASS
- [ ] **Step 5: Commit** — «Compute safe servo ranges and horn re-seating from a measured stop».

---

### Task 8: Редактор поз и зеркало

**Files:**
- Create: `hardware/v2/twin/poser.py`
- Test: `hardware/v2/twin/tests/test_poser.py`

**Interfaces:**
- Consumes: `model.load/set_pose`, `calibrate.safe_range`, `params.CHANNEL`.
- Produces: `poser.format_command(ch: int, fw: int, protocol: "main"|"tester") -> str` (`"5 135\n"` / `"5,135\n"`);
  `poser.mirror_allowed(joint: str, fw: int, safe: dict[str, tuple[int,int]]) -> bool`;
  `poser.export_cpp(frames: list[tuple[list[int], int]], name: str) -> str` (кадр = 8 углов + пауза мс; выдаёт `inline void run<Name>Pose() { … }` с `setServoAngle(<имя>, a);` только для изменившихся серв и `delayWithFace(ms);`);
  `poser.open_mirror(port: str | None) -> Mirror | None` (не открылся — `None` и сообщение, без исключения);
  запуск: `hardware/v2/.venv/bin/mjpython -m twin.poser [--mirror /dev/cu.… --protocol main|tester]`.

- [ ] **Step 1: Тесты.**

```python
def test_format():           assert format_command(5, 135, "main") == "5 135\n" and format_command(5, 135, "tester") == "5,135\n"
def test_blocks_unsafe():    assert not mirror_allowed("R1", 170, {"R1": (5, 155)}) and mirror_allowed("R1", 150, {"R1": (5, 155)})
def test_export():
    src = export_cpp([([90]*8, 300), ([135,45,45,135,0,180,0,180], 500)], "hello")
    assert "inline void runHelloPose()" in src and "setServoAngle(R1, 135);" in src and "delayWithFace(500);" in src
    assert src.count("setServoAngle(R1, 90);") == 1
def test_missing_port():     assert open_mirror("/dev/cu.nope") is None
```

- [ ] **Step 2: Запустить — падают.** → FAIL
- [ ] **Step 3: Реализация.** `mujoco.viewer.launch_passive` с физикой; ползунки — родная панель Control вьюера (управление уже в градусах прошивки, Task 4). Цикл раз в 20 мс: изменившиеся `ctrl` → `round` → при включённом зеркале и `mirror_allowed` — в порт. Клавиши: `K` — записать кадр (пауза — время с прошлого кадра, округлённое до 50 мс), `E` — `export_cpp` в терминал и в `out/poses/<время>.h`, `0` — все в 90, `S` — stand. Контакты, кроме лапа–пол, подсвечиваются (`mjVIS_CONTACTPOINT`), центр масс — сферой на полу.
- [ ] **Step 4: Тесты проходят.** → PASS
- [ ] **Step 5: Ручная проверка.** Окно открывается, ползунок R1 крутит бедро, `E` печатает функцию.
- [ ] **Step 6: Commit** — «Add a pose editor that mirrors sliders to the robot and exports C++».

---

### Task 9: Конвейер, отчёт в git, сверка с ногой

**Files:**
- Create: `hardware/v2/tools/twin.sh`
- Modify: `README.ru.md` — раздел «Цифровой двойник» (как запустить, где отчёт, как пользоваться калибровкой и редактором)
- Commit: `hardware/v2/twin/out/report.md`, `out/calibration.md`, `out/calib/*.png`, `out/videos/*.mp4`, `out/geometry.json`

**Interfaces:**
- Consumes: всё выше.

- [ ] **Step 1: `twin.sh`.** Создаёт окружение, если его нет; дальше по порядку `geometry` → `trace` → `model` → `simulate` (все позы из записей) → `report` → `calibrate`; на первой ошибке печатает шаг и выходит с кодом 1.

Run: `bash hardware/v2/tools/twin.sh`
Expected: последняя строка `готово: hardware/v2/twin/out/report.md`, в отчёте 19 поз (или сломанные — с причиной), `out/videos/` — по mp4 на каждую позу со статусом.

- [ ] **Step 2: Все тесты.** Run: `hardware/v2/.venv/bin/pytest hardware/v2/twin/tests -v` → все PASS.

- [ ] **Step 3: Сверка с напечатанной ногой R1+R3 (вместе с пользователем).** Пользователь ставит по очереди 0/90/180 на бедро и лапу через редактор с зеркалом (`--protocol tester`) и говорит, куда смотрит звено и где упор. Разница с двойником > 5° → поправка в `params.py` (знак, `CAD_POSE_FW`, `SPLINE_TEETH` — пересчитать зубцы на качалке), перегенерировать. Готово, когда расхождение ≤ 5°; известный упор бедра на 155° проверить через `python -m twin.calibrate R1 --stop 155`.

- [ ] **Step 4: Commit** — `twin.sh`, `README.ru.md`, `out/` из списка → «Run the whole twin with one script and add its report».
