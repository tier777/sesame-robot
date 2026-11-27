"""Модель MuJoCo из geometry.json и params: звенья, шарниры по осям серв, MG90S как привод в градусах прошивки."""
from __future__ import annotations

import math
from functools import lru_cache
from xml.sax.saxutils import quoteattr

import mujoco
import numpy as np
import trimesh

from twin import params as P

MM = 0.001


def _fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in np.ravel(v))


def _mass_props(link: str, geom: dict) -> tuple[float, np.ndarray, np.ndarray]:
    """Масса (кг), центр масс (м) и тензор инерции (кг·м², в центре масс) звена в координатах двойника."""
    info = geom["links"][link]
    hulls = [trimesh.load(P.OUT / h) for h in info["hulls"]]
    shape = trimesh.util.concatenate(hulls).convex_hull if link != "torso" else None
    parts = []
    printed_kg = info["printed_volume_cm3"] * 1e-6 * P.DENSITY_PETG * P.INFILL_FACTOR
    servo_kg = P.SERVO_MASS_G / 1000
    if link == "torso":
        body = trimesh.util.concatenate(hulls)
        extra_kg = 4 * servo_kg + (P.DEVKIT_G + P.OLED_G + P.MINI560_G) / 1000
        parts.append(_scaled(body, printed_kg + extra_kg))
        box = trimesh.creation.box(extents=P.BATTERY["size_mm"])
        box.apply_translation(P.BATTERY["center_mm"])
        parts.append(_scaled(box, P.BATTERY["mass_g"] / 1000))
    else:
        kg = printed_kg + (servo_kg if link.startswith("foot") else 0.0)
        parts.append(_scaled(shape, kg))
    mass = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / mass
    inertia = np.zeros((3, 3))
    for m, c, i in parts:
        r = c - com
        inertia += i + m * (r @ r * np.eye(3) - np.outer(r, r))
    return mass, com, inertia


def _scaled(mesh: trimesh.Trimesh, kg: float) -> tuple[float, np.ndarray, np.ndarray]:
    m = mesh.convex_hull if not mesh.is_watertight else mesh
    vol = abs(m.volume)
    m.density = kg / vol                                   # кг/мм³
    return kg, m.center_mass * MM, m.moment_inertia * MM * MM


def build_mjcf(geom: dict) -> str:
    links = geom["links"]
    joints = geom["joints"]
    assets, bodies = [], {}

    def geoms_for(link: str, origin_mm: np.ndarray) -> str:
        info = links[link]
        out = []
        assets.append(f'<mesh name="{link}_vis" file="{info["visual"]}" scale="{MM} {MM} {MM}"/>')
        out.append(f'<geom name="{link}_visual" type="mesh" mesh="{link}_vis" pos="{_fmt(-origin_mm * MM)}" '
                   f'contype="0" conaffinity="0" group="1" rgba="0.92 0.55 0.2 1"/>')
        for i, h in enumerate(info["hulls"]):
            assets.append(f'<mesh name="{link}_hull{i}" file="{h}" scale="{MM} {MM} {MM}"/>')
            out.append(f'<geom name="{link}_hull{i}" type="mesh" mesh="{link}_hull{i}" pos="{_fmt(-origin_mm * MM)}" '
                       f'group="3" rgba="0.3 0.6 1 0.3"/>')
        mass, com, inertia = _mass_props(link, geom)
        out.append(f'<inertial pos="{_fmt(com - origin_mm * MM)}" mass="{mass:.6g}" '
                   f'fullinertia="{_fmt([inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]])}"/>')
        return "\n".join(out)

    def joint_xml(name: str) -> str:
        j = joints[name]
        s = P.joint_sign(name)
        lo, hi = sorted((P.fw_to_rad(name, 0), P.fw_to_rad(name, 180)))
        return (f'<joint name="{name}" type="hinge" axis="{_fmt(j["axis"])}" range="{lo:.6f} {hi:.6f}" '
                f'limited="true" damping="0.01" armature="0.0005"/>')

    feet = []
    for hip in P.HIPS:
        foot = {v: k for k, v in P.PARENT_HIP.items()}[hip]
        ha = np.array(joints[hip]["anchor_mm"])
        fa = np.array(joints[foot]["anchor_mm"])
        feet.append(f'''
      <body name="femur_{hip}" pos="{_fmt(ha * MM)}">
        {joint_xml(hip)}
        {geoms_for(f"femur_{hip}", ha)}
        <body name="foot_{foot}" pos="{_fmt((fa - ha) * MM)}">
          {joint_xml(foot)}
          {geoms_for(f"foot_{foot}", fa)}
        </body>
      </body>''')
    torso = geoms_for("torso", np.zeros(3))

    actuators = []
    for name in P.JOINTS:
        s, cad = P.joint_sign(name), P.CAD_POSE_FW[name]
        g = P.KP * s * math.pi / 180
        actuators.append(
            f'<general name="{name}" joint="{name}" ctrllimited="true" ctrlrange="0 180" forcelimited="true" '
            f'forcerange="{-P.SERVO_TORQUE_NM} {P.SERVO_TORQUE_NM}" gaintype="fixed" gainprm="{g:.8g}" '
            f'biastype="affine" biasprm="{-g * cad:.8g} {-P.KP} {-P.KV}"/>')

    return f'''<mujoco model="sesame">
  <compiler angle="radian" meshdir={quoteattr(str(P.OUT))} autolimits="true"/>
  <option timestep="{P.DT}" integrator="implicitfast"/>
  <default><geom friction="{P.FLOOR_FRICTION} 0.005 0.0001" condim="3"/></default>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256" rgb1="0.85 0.85 0.85" rgb2="0.75 0.75 0.75"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
    {chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="1 1 0.01" material="grid"/>
    <body name="torso" pos="0 0 0">
      <freejoint name="root"/>
      {torso}
      {"".join(feet)}
    </body>
  </worldbody>
  <actuator>
    {chr(10).join(actuators)}
  </actuator>
</mujoco>'''


@lru_cache(maxsize=1)
def _compiled() -> mujoco.MjModel:
    xml = build_mjcf(P.geometry())
    path = P.OUT / "sesame.xml"
    path.write_text(xml)
    return mujoco.MjModel.from_xml_path(str(path))


def load() -> tuple[mujoco.MjModel, mujoco.MjData]:
    m = _compiled()
    return m, mujoco.MjData(m)


def actuator_index(joint: str) -> int:
    return P.CHANNEL[joint]


def is_floor_pair(m: mujoco.MjModel, contact) -> bool:
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    return floor in (contact.geom1, contact.geom2)


def _lowest_z(m: mujoco.MjModel, d: mujoco.MjData) -> float:
    low = math.inf
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_contype[g] == 0:
            continue
        v = m.geom_dataid[g]
        verts = m.mesh_vert[m.mesh_vertadr[v]:m.mesh_vertadr[v] + m.mesh_vertnum[v]]
        z = d.geom_xpos[g][2] + verts @ d.geom_xmat[g].reshape(3, 3)[2]
        low = min(low, float(z.min()))
    return low


def set_pose(m: mujoco.MjModel, d: mujoco.MjData, fw: list[int], on_floor: bool = True) -> None:
    """Суставы в углах прошивки (по каналам), корпус без поворота; on_floor — нижняя точка касается пола."""
    mujoco.mj_resetData(m, d)
    d.qpos[3:7] = [1, 0, 0, 0]
    for name in P.JOINTS:
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = \
            P.fw_to_rad(name, fw[P.CHANNEL[name]])
    d.ctrl[:] = fw
    mujoco.mj_forward(m, d)
    if on_floor:
        d.qpos[2] -= _lowest_z(m, d) + 0.0001   # лапы на 0,1 мм в полу: контакт есть сразу
        mujoco.mj_forward(m, d)
