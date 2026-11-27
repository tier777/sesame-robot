"""Геометрия Sesame из STEP-сборки автора: детали по звеньям, оси валов серв, сетки и выпуклые части."""
from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import coacd
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFPrs import XCAFPrs_DocumentExplorer, XCAFPrs_DocumentExplorerFlags_None

HIPS = ("R1", "R2", "L1", "L2")
FOOT_OF = {"R1": "R3", "R2": "R4", "L1": "L3", "L2": "L4"}
SPLINE_R = (2.40, 2.52)          # шлиц вала SG90/MG90S в модели автора: 60 граней радиусом 2,45–2,48 мм
SKIP = ("Battery",)              # аккумуляторы автора: батарея этой сборки задаётся в params.py
VISUAL_FACES = 20000
HORN = "Pro.2_Pe"                 # подсборка сервы-качалка (внутренний шлиц 2,48 мм)


@dataclass
class Part:
    name: str
    mesh: trimesh.Trimesh          # мм, координаты сборки
    shape: object                  # TopoDS_Shape с положением


@dataclass
class Servo:
    name: str
    anchor: np.ndarray             # точка на оси вала (середина шлица), координаты сборки
    axis: np.ndarray               # единичный вектор от корпуса к валу
    mesh: trimesh.Trimesh


def _name(label) -> str:
    n = TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ""


def _tessellate(shape) -> trimesh.Trimesh:
    BRepMesh_IncrementalMesh(shape, 0.1, False, 0.5, True)
    verts, faces, off = [], [], 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            pts = [tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1)]
            verts += [(p.X(), p.Y(), p.Z()) for p in pts]
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                faces.append((off + a - 1, off + c - 1, off + b - 1) if rev else (off + a - 1, off + b - 1, off + c - 1))
            off += len(pts)
        ex.Next()
    return trimesh.Trimesh(np.array(verts), np.array(faces), process=True)


@lru_cache(maxsize=2)
def _read(step: str) -> tuple[Part, ...]:
    doc = TDocStd_Document(TCollection_ExtendedString("sesame"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(step) != IFSelect_RetDone:
        raise ValueError(f"не читается STEP: {step}")
    reader.Transfer(doc)
    parts = []
    servo = None
    ex = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_None)
    while ex.More():
        node = ex.Current()
        depth = ex.CurrentDepth()
        name = _name(node.Label) or _name(node.RefLabel) or node.Id.ToCString()
        shape = XCAFPrs_DocumentExplorer.FindShapeFromPathId_s(doc, node.Id)
        if depth == 1:
            servo = name if ("SG90" in name and node.IsAssembly) else None
            if not any(s in name for s in SKIP):
                if servo:
                    parts.append(Part(name, _tessellate(_without_horn(doc, node)), shape))
                else:
                    mesh = _tessellate(shape)
                    if len(mesh.faces):
                        parts.append(Part(name, mesh, shape))
        elif depth == 2 and servo and HORN in name:
            # качалка сидит на валу и крутится вместе с тем, что на неё надето
            parts.append(Part(f"{servo}::horn", _tessellate(shape), shape))
        ex.Next()
    return tuple(parts)


def _without_horn(doc, node):
    """Серва без качалки: корпус, вал и винты."""
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound
    comp, builder = TopoDS_Compound(), BRep_Builder()
    builder.MakeCompound(comp)
    sub = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_None)
    prefix = node.Id.ToCString()
    while sub.More():
        n = sub.Current()
        nm = _name(n.Label) or _name(n.RefLabel)
        if sub.CurrentDepth() == 2 and n.Id.ToCString().startswith(prefix) and HORN not in nm:
            builder.Add(comp, XCAFPrs_DocumentExplorer.FindShapeFromPathId_s(doc, n.Id))
        sub.Next()
    return comp


def read_parts(step: Path) -> list[Part]:
    """Детали первого уровня сборки (сервы — целиком), сетки в мм в координатах сборки."""
    return list(_read(str(step)))


def find_servos(parts: list[Part]) -> list[Servo]:
    servos = []
    for p in parts:
        if "SG90" not in p.name or p.name.endswith("::horn"):
            continue
        pts, dirs = [], []
        ex = TopExp_Explorer(p.shape, TopAbs_FACE)
        while ex.More():
            s = BRepAdaptor_Surface(TopoDS.Face(ex.Current()))
            if s.GetType() == GeomAbs_Cylinder and SPLINE_R[0] <= s.Cylinder().Radius() <= SPLINE_R[1]:
                ax = s.Cylinder().Axis()
                d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
                dirs.append(d if not dirs or d @ dirs[0] > 0 else -d)
                umid, vmid = (s.FirstUParameter() + s.LastUParameter()) / 2, (s.FirstVParameter() + s.LastVParameter()) / 2
                q = s.Value(umid, vmid)
                pts.append((q.X(), q.Y(), q.Z()))
            ex.Next()
        if not dirs:
            raise ValueError(f"у сервы {p.name} не найден шлиц вала")
        axis = np.mean(dirs, axis=0)
        axis /= np.linalg.norm(axis)
        pts = np.array(pts)
        # точка на оси: среднее по граням шлица (они равномерно по кругу)
        anchor = pts.mean(axis=0)
        if axis @ (anchor - p.mesh.centroid) < 0:
            axis = -axis
        servos.append(Servo(p.name, anchor, axis, p.mesh))
    return servos


def _line_dist(point: np.ndarray, anchor: np.ndarray, axis: np.ndarray) -> float:
    v = point - anchor
    return float(np.linalg.norm(v - (v @ axis) * axis))


def assign_servos(servos: list[Servo], parts: list[Part], up: np.ndarray | None = None) -> dict[str, str]:
    """Серва → сустав. Бедро: вертикальная ось, ближайшая к детали femur-joint. Лапа: горизонтальная ось,
    корпус ближе всех к детали foot-joint."""
    up = np.array([0.0, 1.0, 0.0]) if up is None else up
    by_name = {}
    for p in parts:
        for key in ("femur", "foot"):
            if key in p.name:
                by_name[p.name.split(":")[0].split("-")[-1]] = p
    vertical = [s for s in servos if abs(s.axis @ up) > 0.9]
    level = [s for s in servos if abs(s.axis @ up) < 0.1]
    out, used = {}, set()
    for hip in HIPS:
        for joint, pool, target in ((hip, vertical, by_name.get(hip)), (FOOT_OF[hip], level, by_name.get(FOOT_OF[hip]))):
            if target is None:
                raise ValueError(f"в сборке нет детали сустава {joint}")
            free = [s for s in pool if s.name not in used]
            if not free:
                raise ValueError(f"не найдена серва сустава {joint}")
            if joint in HIPS:
                best = min(free, key=lambda s: _line_dist(target.mesh.centroid, s.anchor, s.axis))
                dist = _line_dist(target.mesh.centroid, best.anchor, best.axis)
            else:
                best = min(free, key=lambda s: np.linalg.norm(s.mesh.centroid - target.mesh.centroid))
                dist = np.linalg.norm(best.mesh.centroid - target.mesh.centroid)
            if dist > 30:
                raise ValueError(f"не найдена серва сустава {joint}: ближайшая {best.name} в {dist:.0f} мм")
            out[joint] = best.name
            used.add(best.name)
    return out


def _frame(parts: list[Part], servos: dict[str, Servo]) -> tuple[np.ndarray, np.ndarray]:
    """Матрица сборка → двойник (3×3) и начало в координатах сборки. Z — вверх (у Fusion это Y),
    X — к экрану, Y = Z × X; X/Y = 0 — середина осей бёдер, Z = 0 — низ нижней крышки."""
    up = np.array([0.0, 1.0, 0.0])
    hips = np.array([servos[h].anchor for h in HIPS])
    mid = hips.mean(axis=0)
    oled = next(p for p in parts if "OLED" in p.name).mesh.centroid
    fwd = oled - mid
    fwd -= (fwd @ up) * up
    x = np.round(fwd / np.linalg.norm(fwd))        # ось сборки, ближайшая к направлению на экран
    z = up
    y = np.cross(z, x)
    rot = np.array([x, y, z])
    bottom = min(p.mesh.bounds[0] @ up for p in parts if "Bottom-Cover" in p.name)
    origin = mid - (mid @ up) * up + bottom * up
    return rot, origin


def _to_twin(mesh: trimesh.Trimesh, rot: np.ndarray, origin: np.ndarray) -> trimesh.Trimesh:
    m = mesh.copy()
    m.vertices = (m.vertices - origin) @ rot.T
    return m


def _link_of(part: str, servo_joint: dict[str, str]) -> str:
    if part.endswith("::horn"):
        joint = servo_joint[part[:-len("::horn")]]
        hip = joint if joint in HIPS else {v: k for k, v in FOOT_OF.items()}[joint]
        return f"femur_{hip}"
    joint = servo_joint.get(part)
    if joint and joint not in HIPS:
        return f"foot_{joint}"
    for key, prefix in (("femur", "femur"), ("foot", "foot")):
        if key in part:
            return f"{prefix}_{part.split(':')[0].split('-')[-1]}"
    return "torso"


def _hulls(mesh: trimesh.Trimesh, printed: bool) -> list[trimesh.Trimesh]:
    if not printed:
        return [mesh.convex_hull]
    res = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=0.08, max_convex_hull=4)
    return [trimesh.Trimesh(v, f).convex_hull for v, f in res]


def build(step: Path, out_dir: Path) -> dict:
    out_dir = Path(out_dir)
    meshes = out_dir / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    parts = read_parts(step)
    servo_list = find_servos(parts)
    assignment = assign_servos(servo_list, parts)
    by_servo = {s.name: s for s in servo_list}
    servos = {j: by_servo[n] for j, n in assignment.items()}
    rot, origin = _frame(parts, servos)
    servo_joint = {n: j for j, n in assignment.items()}

    links: dict[str, dict] = {}
    groups: dict[str, list[Part]] = {}
    solid = [p for p in parts if "Screw" not in p.name]
    trees = {p.name: cKDTree(p.mesh.vertices) for p in solid}
    for p in parts:
        owner = p.name
        if "Screw" in p.name:   # винт едет с той деталью, которую держит: ближайшая к нему деталь
            owner = min(solid, key=lambda q: trees[q.name].query(p.mesh.centroid)[0]).name
        groups.setdefault(_link_of(owner, servo_joint), []).append(p)
    for link, members in groups.items():
        twin_parts = [(p, _to_twin(p.mesh, rot, origin)) for p in members]
        visual = trimesh.util.concatenate([m for _, m in twin_parts])
        if len(visual.faces) > VISUAL_FACES:
            visual = visual.simplify_quadric_decimation(face_count=VISUAL_FACES)
        visual.export(meshes / f"{link}.stl")
        hull_files, printed_volume = [], 0.0
        for p, m in twin_parts:
            printed = not any(k in p.name for k in ("SG90", "::horn", "Screw", "OLED", "Board", "KCD1", "Display"))
            if printed:
                printed_volume += abs(m.volume) / 1000
            for i, h in enumerate(_hulls(m, printed)):
                f = meshes / f"{link}_{len(hull_files):02d}.stl"
                h.export(f)
                hull_files.append(f"meshes/{f.name}")
        links[link] = {"parts": [p.name for p in members], "visual": f"meshes/{link}.stl",
                       "hulls": hull_files, "printed_volume_cm3": round(printed_volume, 3)}

    joints = {}
    for hip in HIPS:
        foot = FOOT_OF[hip]
        for joint, parent, child in ((hip, "torso", f"femur_{hip}"), (foot, f"femur_{hip}", f"foot_{foot}")):
            s = servos[joint]
            joints[joint] = {"parent": parent, "child": child,
                             "anchor_mm": ((s.anchor - origin) @ rot.T).round(4).tolist(),
                             "axis": (s.axis @ rot.T).round(6).tolist()}
    servo_info = {}
    for joint, s in servos.items():
        m = _to_twin(s.mesh, rot, origin)
        box = m.bounding_box_oriented
        servo_info[joint] = {"case_center_mm": box.centroid.round(4).tolist(),
                             "case_size_mm": np.sort(box.primitive.extents).round(3).tolist(),
                             "case_transform": box.primitive.transform.round(6).tolist(),
                             "link": "torso" if joint in HIPS else f"foot_{joint}"}
    geom = {"frame": {"cad_to_twin": rot.tolist(), "origin_mm": origin.tolist()},
            "links": links, "joints": joints, "servos": servo_info}
    (out_dir / "geometry.json").write_text(json.dumps(geom, indent=1, ensure_ascii=False))
    return geom
