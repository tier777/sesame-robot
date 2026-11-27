# Sesame v2: рама с батареей внутри — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Рама v2 и крышка отсека: отсек 2×14500 внутри, открытые гнёзда бёдер, свободный ход бёдер на всё, что гоняет прошивка, окна облегчения по топооптимизации, места под зарядку и плату защиты; проверено двойником, печать без поддержек.

**Architecture:** Своя SDF-библиотека на numpy строит поле расстояний на сетке, `skimage.measure.marching_cubes` превращает его в STL. Все посадочные размеры берутся из двойника (`twin/out/geometry.json`) и STEP автора; зона хода бёдер — из записей поз прошивки. Двойник получает вариант геометрии `v2` (выходы в `twin/out_v2/`) и прогоняет калибровку и все позы.

**Tech Stack:** Python 3.12 (`hardware/v2/.venv`), numpy, scipy (sparse, ndimage), scikit-image, trimesh, MuJoCo (через двойник), pytest.

**Spec:** `docs/superpowers/specs/2026-11-27-sesame-v2-body-design.md`

## Global Constraints

- Код — `hardware/v2/body/`, тесты — `hardware/v2/body/tests/`, выходы — `hardware/v2/body/out/` (STL, PNG, 3MF в git; промежуточное — нет).
- Оси и единицы — как у двойника: X к мордочке, Y влево, Z вверх; X/Y = 0 — середина осей бёдер, Z = 0 — низ нижней крышки v1; мм.
- Бёдра, оси суставов, верхняя крышка `Top-Cover-Enclosed-v117`, бёдра и лапы v117, прошивка — без изменений.
- Дека: верх на z = 23,0; плита 2,0 мм. Отсек: держатель 58×31×16, зазор 0,4 на сторону, низ держателя z = −2,4, крышка отсека z −4,0…−2,4.
- Стенки 1,6–2,0 мм; внутренние скругления r ≥ 1,5 (плавное объединение k = 1,5); наружные кромки r ≥ 0,8.
- Гнездо сервы: зазор 0,3 мм на сторону; саморезы M2 — пилот 1,7 мм; саморезы верхней крышки и крышки отсека — пилот 1,7, проход 2,3.
- Ход бёдер: пустота = объём, заметаемый бедром и лапой в диапазоне углов, который даёт прошивка во всех позах (по `twin/traces/*.json`), + 1,5 мм.
- Печать декой вниз, без поддержек: грани, смотрящие вниз круче 45° от вертикали, — только стол и мосты ≤ 20 мм.
- Пластик рамы и крышки отсека вместе ≤ 53 см³.
- Габариты плат по умолчанию (до замера): зарядка LX-USC-V2 32×22×8, плата защиты BMS 2S 46×22×5, MPU-6500 25×16×3, Mini560 33×18×5, усилитель MAX98357A 19×18×3, динамик ⌀20×4,5, микрофон INMP441 ⌀14×3 — параметрами в начале `frame_v2.py`.
- Сетка SDF — шаг 0,15 мм для итоговых STL, 0,5 мм в тестах.
- Сообщения, отчёты, README — по-русски.

## Review Focus

- Плата больше заложенного габарита (замер не совпал с карточкой) — сборка падает с понятным сообщением «плата X не входит в гнездо, нужно N мм», а не печатает раму, в которую плата не встанет.
- Прошивка добавила позу или расширила углы — зона хода пересчитывается из записей, а не из зашитых чисел.
- Верхняя крышка v117 не надевается: пересечение с рамой больше 0,05 см³ или отверстия под саморезы разошлись больше 0,2 мм — проверка падает.
- Сетка с дырами или вывернутыми гранями (marching cubes на краю сетки) — проверка watertight падает, STL не пишется.
- Двойник на `v2` находит упор там, где v1 его не было (новая стенка отсека задевает лапу) — раздел «За упором» не пуст, задача не закрыта.

---

### Task 1: SDF-библиотека

**Files:** Create `hardware/v2/body/__init__.py`, `hardware/v2/body/sdf.py`; Test `hardware/v2/body/tests/test_sdf.py`, `hardware/v2/body/tests/conftest.py`; Modify `hardware/v2/requirements.txt` (+ `scikit-image`).

**Interfaces — Produces:**
- `Field = Callable[[np.ndarray (N×3)], np.ndarray (N,)]` — отрицательно внутри.
- `box(size, center=(0,0,0), r=0.0)`, `cylinder(r, h, center=(0,0,0), axis="z")`, `capsule(a, b, r)`, `slab_z(z0, z1)`.
- `union(*f)`, `intersection(*f)`, `difference(a, *b)`, `smooth_union(a, b, k)`, `translate(f, v)`, `from_grid(values, origin, step)` (трилинейная интерполяция по воксельному полю).
- `mesh(f, lo, hi, step) -> trimesh.Trimesh` — считает поле кусками по z (не больше 64 млн точек за раз), `marching_cubes(level=0)`, на выходе замкнутая сетка; бросает `ValueError`, если тело касается края сетки.

- [ ] **Step 1: Тесты.**

```python
def test_box_volume():       m = mesh(box((10,20,30)), (-6,-11,-16), (6,11,16), 0.25); assert m.is_watertight and abs(m.volume - 6000) < 60
def test_hole_diameter():    # дыра 2,0 мм в пластине — после сетки 2,0 ± 0,1
def test_smooth_union_fills_corner():  # в углу двух плит k=1.5 появляется материал, которого нет у обычного union
def test_touching_edge_raises():       with pytest.raises(ValueError, match="край"): mesh(box((10,10,10)), (-5,-5,-5), (5,5,5), 0.5)
def test_from_grid_matches_source():   # поле шара, записанное в сетку и прочитанное from_grid, расходится < step/2
```

- [ ] **Step 2: Запустить — падают** (`hardware/v2/.venv/bin/pytest hardware/v2/body/tests/test_sdf.py -q` → FAIL, нет модуля).
- [ ] **Step 3: Реализация** по интерфейсам; `smooth_union` — полиномиальный smooth-min: `h = clip(0.5 + 0.5*(b-a)/k, 0, 1); a*h + b*(1-h) - k*h*(1-h)` (для `a`,`b` — значения полей).
- [ ] **Step 4: Тесты проходят.**
- [ ] **Step 5: Commit** — «Add a small SDF modelling library for the v2 body».

---

### Task 2: Интерфейсы v1 — что v2 обязан сохранить

**Files:** Create `hardware/v2/body/interfaces.py`; Test `hardware/v2/body/tests/test_interfaces.py`.

**Interfaces:**
- Consumes: `twin.params.geometry()`, `twin.geometry.read_parts`, `_to_twin`.
- Produces: `interfaces.load() -> V1` (кешируется в `body/out/interfaces.json`):
  `V1(deck_top_z: float, cover_screws: list[xy] (4 точки), cover_rim: np.ndarray (полигон низа крышки, xy), cover_usb_window: (x, y0, y1, z0, z1), servos: dict[hip -> Servo(center, axes 3×3, size, flange_holes: list[xyz])], frame_v1_volume_cm3, bottom_v1_volume_cm3, cover_mesh: trimesh)`.

- [ ] **Step 1: Тесты.**

```python
def test_deck_top():          assert load().deck_top_z == pytest.approx(23.0, abs=0.1)
def test_four_cover_screws_symmetric(): s = np.array(load().cover_screws); assert len(s) == 4 and abs(s[:,1].sum()) < 0.5
def test_flange_holes_hit_v1_frame_holes():  # каждая точка отверстия фланца лежит в сквозной дыре рамы v1 (луч по z не встречает пластик в 0,5 мм от оси)
def test_usb_window_at_back():   w = load().cover_usb_window; assert w[0] < -35 and w[2] - w[1] > 9 and w[4] - w[3] > 5
def test_volumes():           v = load(); assert v.frame_v1_volume_cm3 + v.bottom_v1_volume_cm3 == pytest.approx(53.3, abs=0.5)
```

- [ ] **Step 2: FAIL** → **Step 3:** отверстия крышки — сквозные вертикальные дыры ⌀1,5–2,6 мм в раме v1 внутри контура крышки (лучи по z на сетке 0,25 мм в углах); отверстия фланцев — центр корпуса сервы ± 13,9 мм по длинной оси, высота — верх фланца; окно USB — лучи по x через заднюю стенку крышки на сетке y/z 0,5 мм; контур крышки — сечение её сетки на z = 23,5. → **Step 4: PASS** → **Step 5: Commit** — «Extract the v1 interfaces the v2 frame must keep».

---

### Task 3: Зона хода бёдер

**Files:** Create `hardware/v2/body/sweep.py`; Test `hardware/v2/body/tests/test_sweep.py`.

**Interfaces:**
- Consumes: `twin/traces/*.json` (Task 1 двойника), `twin.calibrate._moving_points`, `twin.params.fw_to_rad`.
- Produces: `sweep.commanded_ranges(traces_dir) -> dict[hip, (lo, hi)]` (мин/макс угла каждого бедра по всем позам);
  `sweep.keep_out(step=0.5, margin=1.5) -> Field` (поле расстояния до заметаемого объёма минус margin, через `scipy.ndimage.distance_transform_edt` и `sdf.from_grid`).

- [ ] **Step 1: Тесты.**

```python
def test_ranges_from_traces():   r = commanded_ranges(TRACES); assert r["L2"][1] >= 164 and r["R2"][0] <= 0
def test_keep_out_contains_femur_at_stand():   # точки бедра в stand: поле < 0
def test_keep_out_free_at_body_center():       # точка (0, 0, 10): поле > 0
def test_new_pose_widens_range(tmp_path):     # добавленная поза с R1→60 расширяет диапазон R1 до 60
```

- [ ] **Step 2: FAIL** → **Step 3:** точки бедра и лапы (`_moving_points`) поворачиваются с шагом 2° по диапазону, вокселизуются в общую сетку 0,5 мм; если `twin/traces` нет — `FileNotFoundError` с командой пересборки. → **Step 4: PASS** → **Step 5: Commit** — «Compute the hip sweep the frame must leave empty».

---

### Task 4: Топооптимизация боковой стенки

**Files:** Create `hardware/v2/body/topopt.py`; Test `hardware/v2/body/tests/test_topopt.py`; Output `hardware/v2/body/out/topopt_wall.png`, `out/topopt_wall.npy`.

**Interfaces — Produces:** `topopt.optimize(nx, nz, volfrac=0.4, penal=3.0, rmin=1.5, loads, supports, iters=80) -> np.ndarray (nz×nx, плотности 0…1)` — SIMP, плоское напряжённое состояние, Q4-элементы 1×1 мм, фильтр чувствительности, `scipy.sparse.linalg.spsolve`; `topopt.wall_density() -> np.ndarray` — стенка отсека в виде сбоку (длина = длина отсека, высота = от крышки отсека до деки), опоры — участки под фланцами серв на концах, нагрузки — 7,7 Н вниз посередине по длине, распределено по низу, и 5 Н вверх в центре низа (удар).

- [ ] **Step 1: Тесты.**

```python
def test_cantilever_compliance_drops():     # податливость после 40 итераций меньше, чем у равномерной плиты того же объёма
def test_volume_fraction_kept():            assert abs(optimize(...).mean() - 0.4) < 0.02
def test_wall_symmetric():                  d = wall_density(); assert np.abs(d - d[:, ::-1]).max() < 0.05
```

- [ ] **Step 2: FAIL** → **Step 3:** реализация по интерфейсу; PNG карты плотности. → **Step 4: PASS** → **Step 5: Commit** — «Lay out the bay wall cut-outs by 2D topology optimisation».

---

### Task 5: Рама v2 и крышка отсека

**Files:** Create `hardware/v2/body/frame_v2.py`, `hardware/v2/body/check.py`; Test `hardware/v2/body/tests/test_frame.py`; Output `out/frame_v2.stl`, `out/bay_lid_v2.stl`.

**Interfaces:**
- Consumes: `sdf.*`, `interfaces.load()`, `sweep.keep_out()`, `topopt.wall_density()`.
- Produces: `frame_v2.frame_field() -> Field`, `frame_v2.lid_field() -> Field`, `frame_v2.build(out_dir, step=0.15) -> dict[str, Path]`;
  `check.fits(frame: trimesh, lid: trimesh) -> list[str]` (пустой список — всё сходится; иначе строки по-русски).

Состав рамы (всё в `frame_field`, плавное объединение k = 1,5, потом резкие вычитания):
1. Дека — плита z 21…23 по контуру крышки v1 плюс выносы под фланцы серв.
2. Гнёзда серв: окно 23,4×13,0 в деке; боковые стенки 1,6 мм от деки вниз до z = 15; ниже — открыто.
3. Отсек: стенки 1,6 мм вокруг кармана 58,8×31,8 от z = −2,4 до деки; окна облегчения — где `wall_density() < 0.3`; бобышки ⌀5 под 2 самореза крышки отсека на торцах.
4. Бобышки под 4 самореза верхней крышки — в точках `cover_screws`, от деки вниз на 6 мм.
5. На деке: выступы-упоры под DevKit (по углам, высота 1 мм), площадка MPU-6500 у центра с 2 отверстиями под стяжку, гнездо зарядки у задней стенки (бортики 1,2 мм, вырез под USB-C на оси `cover_usb_window`), прорези под стяжки для Mini560.
6. Звук: в передней стенке — гнездо динамика ⌀20,6×4,8 с бортиком-прижимом и решёткой ⌀1,5 мм перед ним; на деке у передней стенки — место усилителя (бортики 1 мм); у задней стенки — гнездо микрофона ⌀14,6 и отверстие ⌀2 мм в стенке напротив порта.
7. Вычитания: карманы серв, отсек, пилоты саморезов, `keep_out`, отверстия ⌀6 для проводов из отсека на деку (2 шт.), выходы проводов серв со скруглением.
Крышка отсека: пластина 1,6 мм по контуру отсека + 2 уха под саморезы, рёбра 1,2×3 мм крест-накрест.

- [ ] **Step 1: Тесты (на сетке 0,5 мм).**

```python
def test_servo_fits_each_pocket():    # коробка MG90S 22,8×12,4×22,5 + фланец 32,1×12,4×2,5 в позе сервы из двойника не пересекает раму
def test_holder_fits_bay():           # коробка 58×31×16 в отсеке не пересекает раму и крышку
def test_cover_fits():                # пересечение с сеткой крышки v117 < 0,05 см³; 4 пилота совпадают с cover_screws до 0,2 мм
def test_hip_sweep_is_empty():        # точки keep_out(margin=0) не внутри рамы
def test_board_too_big_is_reported(): # с параметром зарядки 40×30 — check.fits говорит «зарядка не входит … нужно … мм»
def test_speaker_grille_open():       # луч из центра гнезда динамика вперёд проходит через решётку, не встречая пластика
def test_watertight_and_light():      assert frame.is_watertight and lid.is_watertight and (frame.volume + lid.volume) / 1000 <= 53
def test_no_overhangs():              # доля площади граней с нормалью вниз круче 45°, кроме z=23 (стол) и мостов ≤ 20 мм, = 0
```

- [ ] **Step 2: FAIL** → **Step 3:** реализация; `build` пишет STL только если `check.fits` пуст и сетки замкнуты. → **Step 4: PASS** → **Step 5: Commit** — «Model the v2 frame and bay lid».

---

### Task 6: Двойник на v2

**Files:** Modify `hardware/v2/twin/params.py` (`OUT` из переменной `SESAME_TWIN_OUT`, `BATTERY` по варианту), `hardware/v2/twin/geometry.py` (+ `build_variant`), `hardware/v2/tools/twin.sh` (ключ `v2`); Test `hardware/v2/twin/tests/test_variant.py`; Output `hardware/v2/twin/out_v2/` (report.md, calibration.md, safe_ranges.json, videos).

**Interfaces — Produces:** `geometry.build_variant(base: Path, replace: dict[str, Path], out_dir: Path) -> dict` — копия `geometry.json` из `base`, в звене `torso` детали `Internal-Frame:1` и `Bottom-Cover:1` заменены на STL v2 (уже в координатах двойника), пересчитаны видимая сетка, оболочки и объём; `params.BATTERY_V2 = {"mass_g": 50, "size_mm": [58,31,16], "center_mm": [0, 0, 5.6]}`; `bash hardware/v2/tools/twin.sh v2` — то же, что без ключа, но в `twin/out_v2`.

- [ ] **Step 1: Тесты.**

```python
def test_variant_swaps_frame(tmp_path):   g = build_variant(OUT, REPLACE, tmp_path); parts = g["links"]["torso"]["parts"]
                                           assert "Internal-Frame:1" not in parts and "frame_v2" in parts
def test_other_links_untouched(tmp_path): # femur_R1 и foot_R3 те же, что в base
def test_battery_moves_down():            # с SESAME_TWIN_OUT=out_v2 центр масс корпуса ниже, чем на v1, больше чем на 10 мм
```

- [ ] **Step 2: FAIL** → **Step 3:** реализация. → **Step 4: PASS**
- [ ] **Step 5: Прогон.** `bash hardware/v2/tools/twin.sh v2` → последняя строка `готово: hardware/v2/twin/out_v2/report.md`; в отчёте раздел «За упором» — «Нет.», stand — «стоит», запас устойчивости «обычно» в каждой позе не меньше, чем в `twin/out/report.md` (v1). Если «За упором» не пуст — вернуться к Task 5 (расширить вырез по `keep_out`, причина — в отчёте), это часть задачи.
- [ ] **Step 6: Commit** — «Check the v2 frame in the digital twin».

---

### Task 7: Плита, картинки, README

**Files:** Create `hardware/v2/body/tools/body.sh`, `hardware/v2/body/render.py`; Modify `README.ru.md` (раздел «v2: батарея внутри»); Output `out/plate_v2.3mf`, `out/renders/*.png`.

- [ ] **Step 1:** `render.py` — картинки: сборка (рама v2 + крышка v117 полупрозрачно + сервы + держатель), рама снизу, разрез по середине; `plate_v2.3mf` — рама декой вниз и крышка отсека, разложены на стол 300×300 (как `sesame-frame-leg-R1-R3.3mf`).
- [ ] **Step 2:** `body.sh` — `topopt` → `frame_v2.build` → `render` → `twin.sh v2`; падает на первой ошибке.

Run: `bash hardware/v2/body/tools/body.sh` → последняя строка `готово: hardware/v2/body/out/plate_v2.3mf`.

- [ ] **Step 3:** README: что печатать (плита v2 + верхняя крышка, бёдра и лапы v117), сборка отсека, проводка с зарядкой и без, как пересобрать после замера плат (параметры в начале `frame_v2.py`).
- [ ] **Step 4: Все тесты.** `hardware/v2/.venv/bin/pytest hardware/v2 -q` → все PASS.
- [ ] **Step 5: Commit** — «Add the v2 print plate, renders and build guide».
