// Крепление открытого отсека 2×AA (две 14500) на спину Sesame, поверх Top-Cover-Enclosed-v117.
// Две детали, обе печатаются без поддержек:
//   saddle — седло: две рейки лежат на крыше верхней крышки, с каждого бока по два пальца
//            с крючками защёлкиваются в вырез внизу боковой стенки крышки (середина корпуса).
//            Между пальцами идут провода серв: они выходят из паза средней лапки рамы прямо под вырезом;
//   tray   — лоток под отсек: прикручивается к седлу 4 саморезами M2×5,
//            отсек вщёлкивается между пружинящими защёлками на торцах.
//
// Оси — как у корпуса Sesame после поворота STL (Y-up → Z-up):
//   X — вдоль корпуса, −X — перед (мордочка), +X — зад (выключатель);
//   Y — поперёк, 0 — середина верхней крышки; Z — вверх, 0 — низ нижней крышки.
// Размеры крышки сняты с hardware/printing/stl/top-covers/Top-Cover-Enclosed-v117.stl.
//
// Выбор детали: openscad -D 'part="saddle"' -o stl/saddle.stl battery-mount.scad
// Полная пересборка STL и картинок: bash hardware/battery-mount/tools/export.sh

part = "assembly";  // assembly | saddle | tray | print | check
$fn = 48;
eps = 0.01;

/* ================= Верхняя крышка Sesame (по STL) ================= */
stl_dir = "../printing/stl/";
cover_y0 = 2.35;      // сдвиг: середина крышки в STL лежит на Z = 2.35 исходных координат
roof_z = 55.6;        // плоская крыша
roof_x = [10.0, 54.0];// плоская часть крыши: за ушками и до заднего ската
roof_half = 11.5;     // полуширина плоской части крыши
ear_z = 60.2;         // верх ушек
wall_half = 18.7;     // наружная полуширина вертикальных боковых стенок
cover_wall = 1.9;     // толщина боковой стенки
notch_x = [17.7, 33.3];  // вырез внизу боковой стенки (с обеих сторон)
notch_top = 30.86;       // верхний край выреза

/* ================= Отсек 2×AA ================= */
holder = [58, 31, 16];   // длина × ширина × высота открытого отсека на две 14500
clr = 0.4;               // зазор отсека в лотке на сторону
cell_d = 14.5;

/* ================= Седло ================= */
ear_gap = 0.6;
plate_t = 2.0;
saddle_z0 = ear_z + ear_gap;        // низ пластины седла: над ушками
rail_y = [4.0, 10.5];               // рейки по крыше, |y|
arm_gap = 0.35;                     // зазор ножки до стенки крышки
arm_t = 2.4;
finger_w = 4.0;                     // два пальца на бок, по краям выреза
finger_x = [notch_x[0] + 1.2, notch_x[1] - 1.2 - finger_w];  // мимо скруглений в углах выреза
wire_gap = [finger_x[0] + finger_w, finger_x[1]];  // между пальцами: над пазом рамы x 23.2..28.3
arm_y0 = wall_half + arm_gap;       // внутренняя грань ножки
hook_tip = wall_half - cover_wall - 0.9;  // крючок заходит за стенку на 0.9
hook_top = notch_top - 0.25;
hook_flat = 0.8;
hook_bot = hook_top - hook_flat - (arm_y0 - hook_tip) * tan(30);  // заход 30°
arm_bot = hook_bot - 0.4;

/* ================= Лоток ================= */
tray_cx = 27;                       // середина отсека над серединой корпуса
tray_floor = 2.4;
side_t = 2.0;
side_h = 8;                         // боковые стенки по пояс банкам
clip_t = 2.2;
clip_w = 12;
lip = 1.2;
pocket = [holder[0] + 2 * clr, holder[1] + 2 * clr];
tray_out = [pocket[0] + 2 * clip_t, pocket[1] + 2 * side_t];
tray_z0 = saddle_z0 + plate_t;
holder_z0 = tray_z0 + tray_floor;
clip_h = holder[2] + 0.3;           // низ зуба защёлки над дном лотка

/* ================= Винты лоток → седло ================= */
screw_x = [14, 44];
screw_y = (rail_y[0] + rail_y[1]) / 2;
pilot_d = 1.7;       // под саморез M2 в седле
pass_d = 2.4;        // проход в лотке
head_d = 4.4;  head_h = 1.6;

/* ================= Проверки ================= */
assert(saddle_z0 > ear_z, "седло ляжет на ушки");
assert(rail_y[1] <= roof_half, "рейки съезжают с плоской крыши");
assert(wire_gap[0] <= 23.2 && wire_gap[1] >= 28.0, "пальцы закрывают паз рамы с проводами");
assert(arm_bot > 22 + 5, "под крючком мало места проводам");
assert(tray_out[1] / 2 < arm_y0 + arm_t, "лоток шире седла");
echo(str("tray x: ", tray_cx - tray_out[0] / 2, "..", tray_cx + tray_out[0] / 2,
         "  holder z: ", holder_z0, "..", holder_z0 + holder[2],
         "  arm length: ", saddle_z0 - arm_bot));

/* ================= Помощники ================= */
module rbox(size, r = 1) {  // прямоугольник со скруглёнными вертикальными рёбрами, от z = 0
  linear_extrude(size[2]) offset(r) square([size[0] - 2 * r, size[1] - 2 * r], center = true);
}

/* ================= Седло ================= */
module hook_profile() {  // в плоскости (y, z), для стороны +Y
  polygon([[arm_y0 + eps, hook_top], [hook_tip, hook_top], [hook_tip, hook_top - hook_flat],
           [arm_y0 + eps, hook_bot]]);
}

module saddle() {
  difference() {
    union() {
      // пластина под лоток
      translate([tray_cx, 0, saddle_z0]) rbox([tray_out[0], tray_out[1], plate_t], 2);
      // рейки по крыше
      for (s = [-1, 1]) translate([roof_x[0], s > 0 ? rail_y[0] : -rail_y[1], roof_z + 0.05])
        cube([roof_x[1] - roof_x[0], rail_y[1] - rail_y[0], saddle_z0 - roof_z]);
      // ножки с крючками
      for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0]) {
        translate([finger_x[0], tray_out[1] / 2 - 1, saddle_z0])
          cube([finger_x[1] + finger_w - finger_x[0], arm_y0 + arm_t - tray_out[1] / 2 + 1, plate_t]);
        for (fx = finger_x) {
          translate([fx, arm_y0, arm_bot]) cube([finger_w, arm_t, saddle_z0 + plate_t - arm_bot]);
          translate([fx, 0, 0]) rotate([90, 0, 90]) linear_extrude(finger_w) hook_profile();
        }
      }
    }
    // скос переднего торца реек под заднюю грань ушек
    for (s = [-1, 1]) translate([roof_x[0], s * (rail_y[0] + rail_y[1]) / 2, roof_z])
      rotate([0, 45, 0]) cube([3, rail_y[1] - rail_y[0] + 2, 3], center = true);
    // отверстия под саморезы из лотка
    for (x = screw_x, s = [-1, 1]) translate([x, s * screw_y, saddle_z0 - 5]) cylinder(d = pilot_d, h = 5 + plate_t + eps);
  }
}

/* ================= Лоток ================= */
module clip() {  // защёлка на торце, для стороны +X, от дна лотка
  translate([pocket[0] / 2, -clip_w / 2, 0]) cube([clip_t, clip_w, clip_h + lip + 1.2]);
  // зуб: плоский низ держит торец отсека, сверху скос 45° — отсек вщёлкивается сверху
  translate([pocket[0] / 2 + eps, clip_w / 2, clip_h]) rotate([90, 0, 0]) linear_extrude(clip_w)
    polygon([[0, 0], [-lip, 0], [-lip, 0.3], [0, 0.3 + lip]]);
}

module tray() {
  translate([tray_cx, 0, tray_z0]) difference() {
    union() {
      rbox([tray_out[0], tray_out[1], tray_floor], 2);
      // боковые стенки во всю длину кармана
      for (s = [-1, 1]) translate([-pocket[0] / 2, s > 0 ? pocket[1] / 2 : -pocket[1] / 2 - side_t, 0])
        cube([pocket[0], side_t, tray_floor + side_h]);
      // защёлки по торцам, углы открыты — туда выходят провода отсека
      for (s = [-1, 1]) mirror([s < 0 ? 1 : 0, 0, 0]) translate([0, 0, tray_floor]) clip();
      for (s = [-1, 1]) mirror([s < 0 ? 1 : 0, 0, 0])
        translate([pocket[0] / 2, -clip_w / 2, 0]) cube([clip_t, clip_w, tray_floor + eps]);
    }
    // винты в седло, головки утоплены
    for (x = screw_x, s = [-1, 1]) translate([x - tray_cx, s * screw_y, 0]) {
      translate([0, 0, -eps]) cylinder(d = pass_d, h = tray_floor + 2 * eps);
      translate([0, 0, tray_floor - head_h]) cylinder(d = head_d, h = head_h + eps);
    }
  }
}

/* ================= Для картинок и проверки ================= */
module sesame_body() {
  translate([0, cover_y0, 0]) rotate([90, 0, 0]) {
    color("#3a3f47") import(str(stl_dir, "Internal-Frame-v121.stl"));
    color("#3a3f47") import(str(stl_dir, "Bottom-Cover-v121.stl"));
    color("#e8892b") import(str(stl_dir, "top-covers/Top-Cover-Enclosed-v117.stl"));
  }
}
module cover_only() {
  translate([0, cover_y0, 0]) rotate([90, 0, 0]) import(str(stl_dir, "top-covers/Top-Cover-Enclosed-v117.stl"));
}
module holder_dummy() {
  translate([tray_cx, 0, holder_z0]) {
    color("#222") difference() {
      rbox([holder[0], holder[1], holder[2]], 1.5);
      translate([0, 0, 1.2]) rbox([holder[0] - 5, holder[1] - 2, holder[2]], 1);
    }
    for (s = [-1, 1]) color("#3b7dd8") translate([-25, s * holder[1] / 4, 1.2 + cell_d / 2])
      rotate([0, 90, 0]) cylinder(d = cell_d, h = 50);
  }
}

if (part == "assembly") {
  sesame_body();
  color("#f2f2f2") saddle();
  color("#9ad14b") tray();
  holder_dummy();
}
// печать: седло пластиной вниз, ножки вверх
if (part == "saddle") translate([0, 0, saddle_z0 + plate_t]) rotate([180, 0, 0]) saddle();
// печать: лоток дном вниз
if (part == "tray") translate([0, 0, -tray_z0]) tray();
// обе детали как лягут на стол
if (part == "print") {
  color("#f2f2f2") translate([0, -30, 0]) translate([0, 0, saddle_z0 + plate_t]) rotate([180, 0, 0]) saddle();
  color("#9ad14b") translate([0, 30, -tray_z0]) tray();
}
// пересечение седла с крышкой: должно быть пустым
if (part == "check") intersection() { saddle(); cover_only(); }
