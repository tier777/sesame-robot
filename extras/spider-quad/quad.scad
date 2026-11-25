// Паучок v1: корпус и лапы четвероногого робота на 8 MG90S 180°.
// Оси: X — вправо, Y — вперёд (к мордочке), Z — вверх. Z = 0 — низ корпуса.
// Пауэрбанк висит под брюхом на двух липучках. Каждая лапа: серва бедра стоит вертикально
// в углу корпуса, кронштейн бедра на её качалке держит серву колена, лапа — на качалке колена.
// Левые лапы — hip_bracket, правые — hip_bracket_mirror; лапа leg одна на все четыре
// (для правой стороны та же деталь, просто перевёрнутая).
// Выбор детали: openscad -D 'part="leg"' -o stl/leg.stl quad.scad

part = "assembly";  // assembly | body_bottom | body_top | hip_bracket | hip_bracket_mirror | leg | head
hip_pose = 0;       // для assembly: угол бёдер в пространстве лапы
knee_pose = 0;      // для assembly: угол коленей
$fn = 48;
eps = 0.01;

/* ================= MG90S (сверить штангенциркулем) ================= */
sv_l = 22.8;  sv_w = 12.4;
sv_shaft_off = 6.0;      // ось вала от ближнего торца корпуса
sv_tab_span = 32.1;
sv_tab_t = 2.5;          // толщина фланца
sv_hole_pitch = 27.8;    // у клонов гуляет — отверстия сделаны пазами ±0.5
sv_screw_d = 1.8;
sv_clr = 0.3;
sv_below_tab = 21;       // от верха фланца до дна корпуса сервы
sv_above_tab = 7.4;      // от верха фланца до торца вала
horn_face = 12.5;        // от верха фланца до внешней грани надетой качалки
horn_span = 31;  horn_hub_d = 7.5;  horn_root_w = 6;  horn_tip_w = 4;  horn_t = 2;  horn_clr = 0.3;
horn_pocket = 1.8;       // насколько качалка утоплена в деталь
hub_web = 1.2;           // перемычка под головкой центрального винта качалки (винт не достаёт — уменьшить)
horn_screw_clr = 2.8;    // отверстие под стержень винта качалки
horn_cbore_d = 6;        // колодец над винтом под отвёртку

sv_case_c = sv_l / 2 - sv_shaft_off;  // центр корпуса сервы от оси вала, в сторону корпуса

/* ================= Пауэрбанк ================= */
pb_t = 22;               // толщина пауэрбанка с запасом; от неё зависит длина лап
pb_gap = 8;              // зазор от пауэрбанка до пола в стойке
pb_max_w = 80;           // самый широкий пауэрбанк, который пройдёт между прорезями
strap_w = 20;            // ширина ремешка-липучки

/* ================= Корпус ================= */
body_w = 96;  body_l = 130;  body_h = 26;
wall = 2;  floor_t = 2;  body_r = 6;
hip_x = 36;  hip_y = 53;               // оси серв бёдер
lid_t = 2;
boss_d = 7;  boss_pilot = 2.6;         // бобышки под саморез M3
boss_pos = [[-body_w / 2 + 5, -25], [body_w / 2 - 5, -25], [-body_w / 2 + 5, 25], [body_w / 2 - 5, 25]];
strap_slot_x = pb_max_w / 2 + 2;       // прорези под липучки — снаружи пауэрбанка
strap_slot_y = [-32, 32];
z_ft = body_h;                          // верх фланца сервы бедра

/* ================= Лапа ================= */
coxa_len = 30;           // от оси бедра до оси колена
plate_t = horn_pocket + hub_web + 1.0; // пластина кронштейна над качалкой бедра
br_wall = 3;             // стенка, на которой висит серва колена
knee_loc_z = plate_t + 1 + sv_w / 2;  // ось колена над низом кронштейна
z_br = z_ft + horn_face - horn_pocket; // низ кронштейна бедра (абсолютно)
knee_z = z_br + knee_loc_z;            // ось колена (абсолютно)
ground_z = -(pb_t + pb_gap);
ground_clear = -ground_z;
leg_t = horn_pocket + hub_web + 1.8;
leg_reach = 25;          // кончик лапы дальше оси колена на столько
leg_drop = knee_z - ground_z;          // и ниже на столько
leg_beam = 8;  toe_r = 4;
leg_elbow = [16, 12];
leg_t0 = sv_tab_t + horn_face - horn_pocket;  // низ лапы от плоскости стенки кронштейна

/* ================= Мордочка ================= */
oled_board = [28, 28];   // плата OLED 0,96″
oled_win = [25, 14];  oled_win_dz = 2;  // окно под стекло, смещение вверх от центра платы (сверить)
head_w = 34;  head_h = 34;  head_d = 12;  head_front = 2;
head_ear = [8, 10, 2.5];               // ушки по бокам, винтами M2 в переднюю стенку корпуса
head_screw_x = head_w / 2 + head_ear[0] / 2;
head_z = head_h / 2;                    // центр мордочки над низом корпуса: верх ниже кронштейнов бёдер
assert(head_z + head_h / 2 < z_br, "мордочка заденет кронштейны передних лап");

/* ================= Проверки ================= */
tower_in = (sv_l - sv_shaft_off + sv_clr + wall) * sin(45);  // насколько гнездо сервы заходит внутрь по осям
assert(ground_clear >= pb_t + 5, "пауэрбанк заденет пол");
assert(hip_y - tower_in >= 32 / 2, "между гнёздами серв не помещается PCA9685");
assert(hip_x - tower_in >= 26 / 2, "между гнёздами серв не помещается PCA9685");
assert(sv_below_tab + floor_t <= z_ft, "серва бедра упирается в дно");
assert(horn_face - horn_t > lid_t + sv_above_tab - sv_tab_t - 4, "качалка бедра задевает крышку");
echo(str("knee_z=", knee_z, " leg_drop=", leg_drop, " ground_clear=", ground_clear,
         " height=", knee_z + sv_w / 2 + 3 - ground_z,
         " span_x=", 2 * (hip_x + (coxa_len + leg_reach) * cos(45))));

/* ================= Помощники ================= */
module rrect(size, r) { translate([r, r]) offset(r) square([size[0] - 2 * r, size[1] - 2 * r]); }
module crect(size, r = 0.01) { translate(-size / 2) rrect(size, r); }
module horn2d() {
  circle(d = horn_hub_d);
  for (s = [-1, 1]) hull() {
    translate([0, -horn_root_w / 2]) square([0.01, horn_root_w]);
    translate([s * (horn_span / 2 - horn_tip_w / 2), 0]) circle(d = horn_tip_w);
  }
}
// Гнездо качалки снизу детали, перемычка, колодец под отвёртку сверху. Толщина детали — total.
module horn_seat(total) {
  translate([0, 0, -eps]) linear_extrude(horn_pocket + eps) offset(horn_clr) horn2d();
  translate([0, 0, -eps]) cylinder(d = horn_screw_clr, h = total + 2 * eps);
  translate([0, 0, horn_pocket + hub_web]) cylinder(d = horn_cbore_d, h = total);
}
// Серва для сборки: ось вала в начале координат на плоскости верха фланца, вал смотрит в +Z,
// корпус уходит в −X.
module servo_dummy() {
  color("#2a4f9a") translate([-sv_case_c, 0, 0]) {
    translate([0, 0, -sv_below_tab / 2]) cube([sv_l, sv_w, sv_below_tab], center = true);
    translate([0, 0, -sv_tab_t / 2]) cube([sv_tab_span, sv_w, sv_tab_t], center = true);
    translate([0, 0, 2]) cube([sv_l, sv_w, 4], center = true);
  }
  color("#2a4f9a") cylinder(d = 5, h = sv_above_tab);
  color("white") translate([0, 0, horn_face - horn_t]) linear_extrude(horn_t) horn2d();
}

/* ================= Корпус ================= */
legs = [[-1, 1, 135], [1, 1, 45], [-1, -1, 225], [1, -1, 315]];  // FL, FR, RL, RR: знак X, знак Y, угол лапы
function hip_at(l) = [l[0] * hip_x, l[1] * hip_y];

// Гнездо сервы бедра в локальных осях: X — по диагонали наружу, вал в начале координат.
module hip_pocket2d(grow) {
  translate([-sv_case_c, 0]) square([sv_l + 2 * grow, sv_w + 2 * grow], center = true);
}
module hip_tower2d() {
  translate([-sv_case_c, 0]) square([sv_tab_span + 2 * wall, sv_w + 2 * wall + 2], center = true);
}
module at_hips() {
  for (l = legs) translate(hip_at(l)) rotate(l[2]) children();
}
module body_outline() { crect([body_w, body_l], body_r); }

module body_bottom() {
  difference() {
    union() {
      // дно + стенки
      linear_extrude(floor_t) body_outline();
      linear_extrude(body_h) difference() { body_outline(); offset(-wall) body_outline(); }
      // башни серв бёдер, обрезанные по контуру корпуса
      intersection() {
        linear_extrude(z_ft) at_hips() hip_tower2d();
        linear_extrude(body_h) body_outline();
      }
      // бобышки крышки
      for (p = boss_pos) translate(p) cylinder(d = boss_d, h = body_h);
    }
    // гнёзда серв: корпус сервы проходит до дна, фланцы ложатся на башню
    at_hips() {
      translate([0, 0, floor_t]) linear_extrude(body_h) hip_pocket2d(sv_clr);
      for (s = [-1, 1]) translate([-sv_case_c + s * sv_hole_pitch / 2, 0, z_ft - 8])
        linear_extrude(8 + eps) hull() for (d = [-0.5, 0.5]) translate([d, 0]) circle(d = sv_screw_d);
    }
    // саморезы крышки
    for (p = boss_pos) translate([p[0], p[1], body_h - 10]) cylinder(d = boss_pilot, h = 10 + eps);
    // прорези под липучки пауэрбанка
    for (sx = [-1, 1], y = strap_slot_y) translate([sx * strap_slot_x, y, -eps])
      linear_extrude(floor_t + 2 * eps) crect([3, strap_w + 2]);
    // кабель от пауэрбанка: в дне сзади по центру
    translate([0, -body_l / 2 + 14, -eps]) linear_extrude(floor_t + 2 * eps) crect([14, 8], 2);
    // кабель от пауэрбанка к ESP — запасное отверстие спереди
    translate([0, body_l / 2 - 14, -eps]) linear_extrude(floor_t + 2 * eps) crect([14, 8], 2);
    // крепление мордочки и провода к ней
    for (sx = [-1, 1]) translate([sx * head_screw_x, body_l / 2 + eps, head_z])
      rotate([90, 0, 0]) cylinder(d = 1.6, h = 8);
    translate([0, body_l / 2 - wall / 2, head_z + 4]) cube([12, wall + 2, 6], center = true);
  }
}

module body_top() {
  difference() {
    union() {
      linear_extrude(lid_t) body_outline();
      // бортик внутрь корпуса, чтобы крышка не ёрзала
      translate([0, 0, -2]) linear_extrude(2 + eps) difference() {
        offset(-wall - 0.3) body_outline();
        offset(-wall - 1.8) body_outline();
      }
    }
    // верх серв бёдер торчит сквозь крышку
    translate([0, 0, -3]) linear_extrude(lid_t + 4) at_hips() hip_tower2d();
    // винты M3
    for (p = boss_pos) translate([p[0], p[1], -3]) cylinder(d = 3.4, h = lid_t + 4);
    // провода серв коленей
    for (l = legs) translate([l[0] * (hip_x - 18), l[1] * (hip_y - 24), -3])
      linear_extrude(lid_t + 4) crect([10, 5], 2);
    // облегчение и окно к электронике
    translate([0, 0, -3]) linear_extrude(lid_t + 4) crect([40, 50], 6);
  }
}

/* ================= Кронштейн бедра ================= */
// Локальные оси: X — наружу вдоль лапы (r), Y — по касательной (t), Z — вверх, Z = 0 — низ.
// Ось бедра в начале координат, ось колена — в [coxa_len, *, knee_loc_z] вдоль Y.
// Серва колена висит фланцами на стенке Y = 0, корпус уходит в −Y, вал смотрит в +Y.
kn_case_r0 = coxa_len - sv_l + sv_shaft_off;  // ближний к бедру торец сервы колена
kn_wall_r = [kn_case_r0 - sv_clr - (sv_tab_span - sv_l) / 2 - 3.5, coxa_len + sv_shaft_off + (sv_tab_span - sv_l) / 2 + 3];
kn_plate_t = [-(sv_below_tab - sv_tab_t) - 2.5, horn_hub_d / 2 + 2];
assert(kn_wall_r[0] > horn_cbore_d / 2 + 0.5, "стенка кронштейна закрывает винт качалки бедра");

module hip_bracket() {
  difference() {
    union() {
      // пластина на качалке бедра, под серву колена
      hull() {
        translate([0, 0, 0]) linear_extrude(plate_t) offset(2) horn2d();
        translate([kn_wall_r[0], kn_plate_t[0], 0]) cube([kn_wall_r[1] - kn_wall_r[0], kn_plate_t[1] - kn_plate_t[0], plate_t]);
      }
      // стенка под фланцы сервы колена
      translate([kn_wall_r[0], -br_wall, 0]) cube([kn_wall_r[1] - kn_wall_r[0], br_wall, knee_loc_z + sv_w / 2 + 3]);
    }
    horn_seat(plate_t);
    // окно в стенке: корпус сервы колена проходит насквозь
    translate([coxa_len - sv_case_c, 0, knee_loc_z]) cube([sv_l + 2 * sv_clr, 2 * br_wall + 2, sv_w + 2 * sv_clr], center = true);
    // пазы под саморезы фланцев
    for (s = [-1, 1]) translate([coxa_len - sv_case_c + s * sv_hole_pitch / 2, 1, knee_loc_z]) rotate([90, 0, 0])
      linear_extrude(br_wall + 2) hull() for (d = [-0.5, 0.5]) translate([d, 0]) circle(d = sv_screw_d);
  }
}

/* ================= Лапа ================= */
// Плоская деталь: X — наружу от оси колена, Y — вверх, толщина по Z. Ось колена в начале координат.
module leg2d() {
  hull() { offset(2) horn2d(); translate(leg_elbow) circle(d = leg_beam); }
  hull() { translate(leg_elbow) circle(d = leg_beam); translate([leg_reach, -leg_drop + toe_r]) circle(r = toe_r); }
}
module leg() {
  difference() {
    linear_extrude(leg_t) leg2d();
    horn_seat(leg_t);
  }
}

/* ================= Мордочка ================= */
// Печатается лицом вниз: Z = 0 — лицевая грань с окном под экран. Сзади открыта:
// плату OLED вставляют сзади, а прижимает её передняя стенка корпуса.
module head() {
  difference() {
    union() {
      linear_extrude(head_d) crect([head_w, head_h], 4);
      for (sx = [-1, 1]) hull() {
        translate([sx * head_screw_x, 0, head_d - head_ear[2] / 2]) cube(head_ear, center = true);
        translate([sx * (head_w / 2 - 1), 0, head_d / 2]) cube([2, head_ear[1], head_d], center = true);
      }
    }
    translate([0, oled_win_dz, -eps]) linear_extrude(head_front + 2 * eps) crect(oled_win, 1);
    translate([0, 0, head_front]) linear_extrude(head_d) crect(oled_board + [0.6, 0.6], 1);
    for (sx = [-1, 1]) translate([sx * head_screw_x, 0, -eps]) cylinder(d = 2.2, h = head_d + 1);
  }
}

/* ================= Сборка ================= */
module mirror_if(m) { if (m) mirror([0, 1, 0]) children(); else children(); }

module leg_assembly(l, hip_deg, knee_deg) {
  left = l[0] < 0;
  hip_sign = left ? -1 : 1;  // бедро > 0 — лапа к носу: у левых по часовой, у правых против
  translate([hip_at(l)[0], hip_at(l)[1], 0]) rotate(l[2]) {
    translate([0, 0, z_ft]) servo_dummy();
    rotate(hip_sign * hip_deg) translate([0, 0, z_br]) mirror_if(!left) {
      color("#e07a2e") hip_bracket();
      // серва колена: вал по +Y, верх фланца на плоскости Y = sv_tab_t
      translate([coxa_len, sv_tab_t, knee_loc_z]) rotate([-90, 0, 0]) servo_dummy();
      // лапа на качалке колена; колено > 0 поднимает кончик
      translate([coxa_len, leg_t0, knee_loc_z]) rotate([0, -knee_deg, 0]) mirror([0, 1, 0]) rotate([90, 0, 0])
        color("#e07a2e") leg();
    }
  }
}

module assembly() {
  color("#e07a2e") body_bottom();
  color("#f09a4e") translate([0, 0, body_h]) body_top();
  color("#e07a2e") translate([0, body_l / 2 + head_d, head_z]) rotate([90, 0, 0]) head();
  color("#333") translate([0, 0, -pb_t / 2]) cube([68, 140, pb_t - 1], center = true);
  for (l = legs) leg_assembly(l, hip_pose, knee_pose);
  color("#888", 0.3) translate([0, 0, ground_z - 0.5]) cube([320, 320, 1], center = true);
}

if (part == "assembly") assembly();
else if (part == "body_bottom") body_bottom();
else if (part == "body_top") translate([0, 0, lid_t]) rotate([180, 0, 0]) body_top();
else if (part == "hip_bracket") hip_bracket();
else if (part == "hip_bracket_mirror") mirror([0, 1, 0]) hip_bracket();
else if (part == "leg") leg();
else if (part == "head") head();
