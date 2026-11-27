"""Картинки сборки v2 и плита для печати (3MF на стол K1 Max 300×300).  python -m body.render"""
from __future__ import annotations

import subprocess
import zipfile
from pathlib import Path

import numpy as np
import trimesh

OUT = Path(__file__).parent / "out"
BED = 300.0


def print_pose(m: trimesh.Trimesh, deck_down: bool) -> trimesh.Trimesh:
    """Рама печатается декой вниз (поворот на 180° вокруг X), крышка отсека — как стоит. Низ — на z = 0."""
    m = m.copy()
    if deck_down:
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    m.apply_translation([0, 0, -m.bounds[0][2]])
    return m


def plate_3mf(objects: dict[str, trimesh.Trimesh], path: Path, gap: float = 12.0) -> Path:
    """Объекты в ряд по X по центру стола; каждый — отдельный объект 3MF."""
    widths = [m.extents[0] for m in objects.values()]
    x = BED / 2 - (sum(widths) + gap * (len(widths) - 1)) / 2
    xml = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
           "<resources>"]
    for i, (name, m) in enumerate(objects.items(), 1):
        m = m.copy()
        m.apply_translation([x - m.bounds[0][0], BED / 2 - m.centroid[1], -m.bounds[0][2]])
        x += m.extents[0] + gap
        xml.append(f'<object id="{i}" name="{name}" type="model"><mesh><vertices>')
        xml += [f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in m.vertices]
        xml.append("</vertices><triangles>")
        xml += [f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces]
        xml.append("</triangles></mesh></object>")
    xml += ["</resources><build>"] + [f'<item objectid="{i}"/>' for i in range(1, len(objects) + 1)] + ["</build></model>"]
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model", "\n".join(xml))
    return path


def renders() -> list[Path]:
    """Сборка (рама v2, крышка v117 полупрозрачно, сервы, держатель), вид снизу и разрез по середине — через OpenSCAD."""
    from body.interfaces import load
    tmp = OUT / "render_parts"
    tmp.mkdir(parents=True, exist_ok=True)
    v1 = load()
    v1.cover_mesh.export(tmp / "cover.stl")
    trimesh.util.concatenate([s.case for s in v1.servos.values()]).export(tmp / "servos.stl")
    holder = trimesh.creation.box((58, 31, 16))
    holder.apply_translation([0, 0, -2.4 + 8])
    holder.export(tmp / "holder.stl")
    scad = tmp / "v2.scad"
    scad.write_text(f'''view = "assembly";
module frame() color("#e8892b") import("{OUT / 'frame_v2.stl'}");
module lid() color("#9ad14b") import("{OUT / 'bay_lid_v2.stl'}");
module rest() {{ color("#3b7dd8") import("{tmp / 'servos.stl'}"); color("#222") import("{tmp / 'holder.stl'}"); }}
if (view == "assembly") {{ frame(); lid(); rest(); color("#cccccc", 0.35) import("{tmp / 'cover.stl'}"); }}
if (view == "bottom") {{ frame(); rest(); }}
if (view == "section") intersection() {{ union() {{ frame(); lid(); rest(); }} translate([-60, 0, -20]) cube([120, 60, 80]); }}
''')
    out = []
    for view, cam in (("assembly", "0,0,20,60,0,35,260"), ("bottom", "0,0,8,125,0,35,240"), ("section", "0,0,10,80,0,0,200")):
        dst = OUT / "renders" / f"{view}.png"
        dst.parent.mkdir(exist_ok=True)
        subprocess.run(["openscad", "--render", "--colorscheme=Tomorrow", "--imgsize=1200,900", f"--camera={cam}",
                        "-D", f'view="{view}"', "-o", str(dst), str(scad)], check=True, capture_output=True)
        out.append(dst)
    return out


def main() -> None:
    frame = trimesh.load(OUT / "frame_v2.stl")
    lid = trimesh.load(OUT / "bay_lid_v2.stl")
    plate = plate_3mf({"frame_v2": print_pose(frame, True), "bay_lid_v2": print_pose(lid, False)}, OUT / "plate_v2.3mf")
    for p in renders():
        print(f"картинка: {p.relative_to(OUT.parent)}")
    print(f"готово: {plate.relative_to(OUT.parents[2])}")


if __name__ == "__main__":
    main()
