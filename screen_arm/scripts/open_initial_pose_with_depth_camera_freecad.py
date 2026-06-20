"""Open the depth-camera screen arm model in FreeCAD at the test initial pose.

This script reads generated/urdf/face_screen_support_arm_depth_camera.urdf,
loads every visual STL, applies URDF colors, evaluates the same initial joint
pose used by the MATLAB demos, and opens/saves a FreeCAD scene for screenshots.
"""

import math
import os
import xml.etree.ElementTree as ET

import FreeCAD as App
import Mesh

try:
    import FreeCADGui as Gui
except Exception:
    Gui = None


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(ROOT, "generated", "urdf", "face_screen_support_arm_depth_camera.urdf")
OUTPUT_PATH = os.path.join(ROOT, "generated", "freecad_initial_pose_depth_camera.FCStd")

# Same pose as test_v4/demo_face_pose_screen_arm_realtime_servo_udp_avatar.m:
# displayPoseToConfig([0, -120, 120, 30, 0, 0])
JOINT_VALUES = {
    "joint1_base_yaw": math.radians(0.0),
    "joint2_shoulder_pitch": math.radians(-120.0),
    "joint3_elbow_pitch": math.radians(120.0),
    "joint4_telescopic": 0.030,
    "joint5_screen_pan": math.radians(0.0),
    "joint6_screen_pitch": math.radians(0.0),
}

FALLBACK_COLOR = (0.78, 0.78, 0.76, 1.0)


def parse_vec(text, default):
    if not text:
        return default
    return [float(item) for item in text.split()]


def eye():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def translation(xyz):
    m = eye()
    m[0][3], m[1][3], m[2][3] = xyz
    return m


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    return matmul(matmul(rot_z(yaw), rot_y(pitch)), rot_x(roll))


def axis_rotation(axis, angle):
    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-12:
        return eye()
    x, y, z = x / length, y / length, z / length
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s, 0.0],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s, 0.0],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def joint_origin_matrix(joint):
    origin = joint.find("origin")
    if origin is None:
        return eye()
    xyz = parse_vec(origin.get("xyz"), [0.0, 0.0, 0.0])
    rpy = parse_vec(origin.get("rpy"), [0.0, 0.0, 0.0])
    return matmul(translation(xyz), rpy_matrix(rpy))


def visual_origin_matrix(visual):
    origin = visual.find("origin")
    if origin is None:
        return eye()
    xyz = parse_vec(origin.get("xyz"), [0.0, 0.0, 0.0])
    rpy = parse_vec(origin.get("rpy"), [0.0, 0.0, 0.0])
    return matmul(translation(xyz), rpy_matrix(rpy))


def joint_motion_matrix(joint):
    joint_type = joint.get("type", "fixed")
    value = JOINT_VALUES.get(joint.get("name", ""), 0.0)
    axis_elem = joint.find("axis")
    axis = parse_vec(axis_elem.get("xyz") if axis_elem is not None else None, [1.0, 0.0, 0.0])
    if joint_type in ("revolute", "continuous"):
        return axis_rotation(axis, value)
    if joint_type == "prismatic":
        return translation([axis[0] * value, axis[1] * value, axis[2] * value])
    return eye()


def placement_from_matrix(m):
    matrix = App.Matrix(
        m[0][0], m[0][1], m[0][2], m[0][3],
        m[1][0], m[1][1], m[1][2], m[1][3],
        m[2][0], m[2][1], m[2][2], m[2][3],
        m[3][0], m[3][1], m[3][2], m[3][3],
    )
    return App.Placement(matrix)


def scale_mesh(mesh, scale):
    matrix = App.Matrix()
    matrix.A11 = scale[0]
    matrix.A22 = scale[1]
    matrix.A33 = scale[2]
    mesh.transform(matrix)


def safe_name(text):
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in text)


def load_urdf():
    tree = ET.parse(URDF_PATH)
    robot = tree.getroot()
    materials = {}
    for material in robot.findall("material"):
        color = material.find("color")
        if color is not None:
            materials[material.get("name")] = tuple(parse_vec(color.get("rgba"), list(FALLBACK_COLOR)))
    links = {link.get("name"): link for link in robot.findall("link")}
    child_joint = {}
    children = set()
    for joint in robot.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        child_joint[child] = (parent, joint)
        children.add(child)
    roots = [name for name in links if name not in children]
    root = roots[0] if roots else next(iter(links))
    return robot, materials, links, child_joint, root


def link_transform(name, child_joint, cache):
    if name in cache:
        return cache[name]
    if name not in child_joint:
        cache[name] = eye()
        return cache[name]
    parent, joint = child_joint[name]
    parent_tf = link_transform(parent, child_joint, cache)
    cache[name] = matmul(matmul(parent_tf, joint_origin_matrix(joint)), joint_motion_matrix(joint))
    return cache[name]


def color_for_visual(visual, materials):
    material = visual.find("material")
    if material is None:
        return FALLBACK_COLOR
    inline = material.find("color")
    if inline is not None:
        return tuple(parse_vec(inline.get("rgba"), list(FALLBACK_COLOR)))
    return materials.get(material.get("name"), FALLBACK_COLOR)


def mesh_path_from_visual(visual):
    mesh_elem = visual.find("./geometry/mesh")
    if mesh_elem is None:
        return None, [1.0, 1.0, 1.0]
    filename = mesh_elem.get("filename")
    scale = parse_vec(mesh_elem.get("scale"), [1.0, 1.0, 1.0])
    return os.path.abspath(os.path.join(os.path.dirname(URDF_PATH), filename)), scale


def apply_mesh_color(obj, color):
    view = getattr(obj, "ViewObject", None)
    if view is None:
        return
    rgb = tuple(color[:3])
    try:
        view.ShapeColor = rgb
    except Exception:
        pass
    try:
        view.MeshColor = rgb
    except Exception:
        pass
    try:
        view.Transparency = max(0, min(100, int(round((1.0 - color[3]) * 100))))
    except Exception:
        pass
    try:
        view.DisplayMode = "Shaded"
    except Exception:
        pass


def build_scene():
    robot, materials, links, child_joint, _ = load_urdf()
    try:
        old = App.getDocument("screen_arm_initial_pose")
    except Exception:
        old = None
    if old is not None:
        App.closeDocument(old.Name)
    doc = App.newDocument("screen_arm_initial_pose")
    cache = {}

    for link_name, link in links.items():
        link_tf = link_transform(link_name, child_joint, cache)
        for idx, visual in enumerate(link.findall("visual")):
            mesh_path, scale = mesh_path_from_visual(visual)
            if mesh_path is None or not os.path.isfile(mesh_path):
                continue
            mesh = Mesh.Mesh(mesh_path)
            scale_mesh(mesh, scale)
            name = visual.get("name") or os.path.splitext(os.path.basename(mesh_path))[0]
            obj = doc.addObject("Mesh::Feature", safe_name("{}_{}".format(link_name, name)))
            obj.Mesh = mesh
            obj.Placement = placement_from_matrix(matmul(link_tf, visual_origin_matrix(visual)))
            apply_mesh_color(obj, color_for_visual(visual, materials))

    doc.recompute()
    try:
        doc.saveAs(OUTPUT_PATH)
    except Exception as exc:
        print("Warning: failed to save FreeCAD scene: {}".format(exc))

    if Gui is not None:
        try:
            Gui.ActiveDocument = Gui.getDocument(doc.Name)
        except Exception:
            pass
        try:
            Gui.activateWorkbench("MeshWorkbench")
        except Exception:
            pass
        try:
            Gui.SendMsgToActiveView("ViewFit")
            Gui.runCommand("Std_ViewIsometric", 0)
        except Exception:
            pass
        try:
            view = Gui.ActiveDocument.ActiveView
            view.setCameraType("Perspective")
            view.viewIsometric()
            view.fitAll()
        except Exception:
            pass

    print("Opened initial-pose screen arm with depth camera.")
    print("Saved scene: {}".format(OUTPUT_PATH))


build_scene()
