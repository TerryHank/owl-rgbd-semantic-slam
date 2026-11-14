"""Build the display meshes and a visual-only variant of the upstream 2WD URDF."""
from collections import defaultdict
from math import cos, sin, pi
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import collada
import numpy as np

ws = Path.home() / "linorobot2_ws"
out = ws / "display"
out.mkdir(exist_ok=True)
parts = defaultdict(list)
colors = {"graphite": (.075, .092, .11), "rubber": (.034, .039, .044),
          "blue": (.035, .30, .52), "silver": (.64, .70, .73),
          "amber": (.98, .52, .055), "glass": (.06, .11, .17)}


def shell(color, x, y, levels, wheel=False):
    """Loft rounded rectangular sections; each level is z, length, width, radius."""
    rings = []
    for z, length, width, radius in levels:
        ring = []
        for corner, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
            for step in range(9):
                a = (corner + step / 8) * pi / 2
                xx = x + sx * (length / 2 - radius) + radius * cos(a)
                yy = y + sy * (width / 2 - radius) + radius * sin(a)
                ring.append((xx, -z, yy) if wheel else (xx, yy, z))
        rings.append(ring)
    tris = parts[color]
    for lower, upper in zip(rings, rings[1:]):
        for i in range(len(lower)):
            j = (i + 1) % len(lower)
            tris.extend(((lower[i], lower[j], upper[j]), (lower[i], upper[j], upper[i])))
    for ring, reverse in ((rings[0], True), (rings[-1], False)):
        center = np.mean(ring, axis=0)
        for i, p in enumerate(ring):
            tri = (center, p, ring[(i + 1) % len(ring)])
            tris.append(tri[::-1] if reverse else tri)


def band(color, x, y, bottom, top, length, width, radius):
    shell(color, x, y, [(z, length, width, radius) for z in (bottom, top)])


def save(name):
    doc = collada.Collada()
    doc.assetInfo.upaxis = "Z_UP"
    nodes = []
    for color, triangles in parts.items():
        points = np.asarray(triangles)
        normals = np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0])
        keep = np.linalg.norm(normals, axis=1) > 1e-12
        points, normals = points[keep], normals[keep]
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        pos = collada.source.FloatSource(color+"-p", points.flatten(), ("X","Y","Z"))
        norm = collada.source.FloatSource(color+"-n", normals.flatten(), ("X","Y","Z"))
        geom = collada.geometry.Geometry(doc, color+"-g", color, [pos, norm])
        inputs = collada.source.InputList()
        inputs.addInput(0, "VERTEX", "#"+pos.id)
        inputs.addInput(1, "NORMAL", "#"+norm.id)
        idx = np.column_stack((np.arange(len(points)*3), np.repeat(np.arange(len(points)), 3)))
        geom.primitives.append(geom.createTriangleSet(idx.flatten(), inputs, "surface"))
        doc.geometries.append(geom)
        effect = collada.material.Effect(color+"-e", [], "phong", diffuse=(*colors[color],1), specular=(.16,.16,.16,1), shininess=24.)
        mat = collada.material.Material(color+"-m", color, effect)
        doc.effects.append(effect)
        doc.materials.append(mat)
        nodes.append(collada.scene.GeometryNode(geom, [collada.scene.MaterialNode("surface", mat, [])]))
    scene = collada.scene.Scene("scene", [collada.scene.Node("robot", children=nodes)])
    doc.scenes.append(scene)
    doc.scene = scene
    doc.write(str(out / (name+".dae")))
    parts.clear()


shell("graphite", 0, 0, [(-.038,.235,.202,.032),(-.020,.28,.224,.042),(.013,.282,.224,.042)])
band("rubber",0,0,.009,.021,.294,.237,.041)
shell("blue",0,0,[(.021,.28,.226,.039),(.060,.27,.218,.038),(.077,.228,.192,.038)])
band("silver",-.012,0,.077,.080,.193,.155,.030)
for i in range(6):
    band("graphite",-.076+i*.010,0,.080,.0815,.004,.075,.0018)
for side in (-1,1):
    band("amber",0,side*.111,.027,.035,.128,.006,.002)
    band("graphite",side*.136,0,.025,.065,.023,.141,.006)
    band("graphite",.088,side*.050,.078,.258,.024,.018,.006)
shell("graphite",.1,0,[(.251,.061,.124,.018),(.268,.098,.136,.023),(.280,.098,.136,.023)])
save("chassis")

def disc(color, radius, widths):
    shell(color,0,0,[(z,2*r,2*r,r) for z,r in zip(widths,radius)],wheel=True)

disc("rubber",[.039,.045,.045,.039],[-.013,-.009,.009,.013])
for sign in (-1,1):
    disc("silver",[.028,.028],sorted([sign*.013,sign*.014]))
    disc("graphite",[.013,.013],sorted([sign*.014,sign*.016]))
save("wheel")

src = ws / "src/linorobot2/linorobot2_description/urdf/robots/2wd.urdf.xacro"
robot = ET.fromstring(subprocess.check_output(["xacro", str(src)]))
for link, mesh in (("base_link","chassis"),("left_wheel_link","wheel"),("right_wheel_link","wheel")):
    visual = robot.find(f"link[@name='{link}']/visual")
    visual.clear()
    ET.SubElement(ET.SubElement(visual,"geometry"),"mesh",filename=(out / (mesh+".dae")).as_uri())
    for gazebo in robot.findall(f"gazebo[@reference='{link}']"):
        for override in gazebo.findall("visual"):
            gazebo.remove(override)
ET.ElementTree(robot).write(out / "2wd.urdf", encoding="utf-8", xml_declaration=True)

world = ET.parse(ws / "worlds/playground.world")
world.find(".//gui/camera/pose").text = "0.58 -0.76 0.49 0 0.35 2.22"
world.write(out / "playground.world", encoding="utf-8", xml_declaration=True)
