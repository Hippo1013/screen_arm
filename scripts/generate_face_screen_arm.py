import os

import FreeCAD as App
import MeshPart
import Part


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR) if os.path.basename(SCRIPT_DIR).lower() == "scripts" else SCRIPT_DIR
OUT = os.path.join(ROOT, "generated")
MESH_DIR = os.path.join(OUT, "meshes")
VISUAL_DIR = os.path.join(OUT, "visuals")
STEP_DIR = os.path.join(OUT, "step")
URDF_DIR = os.path.join(OUT, "urdf")


P = {
    "base_radius": 80.0,
    "base_height": 45.0,
    "mast_radius": 38.0,
    "mast_height": 120.0,
    "shoulder_height": 160.0,
    "upper_len": 280.0,
    "forearm_len": 240.0,
    "tube_insert": 240.0,
    "tube_out": 85.0,
    "tube_travel": 280.0,
    "pan_mount_z": 24.0,
    "pan_pitch_x": 70.0,
    "pan_pitch_z": 42.0,
    "screen_center_x": 52.0,
    "screen_center_z": 0.0,
    "arm_width": 52.0,
    "arm_height": 42.0,
    "joint_radius": 34.0,
    "joint_width": 62.0,
    "wrist_gap": 0.0,
    "screen_width": 531.0,
    "screen_height": 299.0,
    "screen_thick": 14.0,
}

DEFAULT_COLOR = (0.78, 0.78, 0.76, 1.0)
JOINT_BLUE = (135.0 / 255.0, 206.0 / 255.0, 235.0 / 255.0, 1.0)
SCREEN_BLACK = (0.0, 0.0, 0.0, 1.0)
CENTER_RED = (1.0, 0.0, 0.0, 1.0)

DISPLAY_COLORS = {}


def ensure_dirs():
    for path in (OUT, MESH_DIR, VISUAL_DIR, STEP_DIR, URDF_DIR):
        os.makedirs(path, exist_ok=True)


def apply_view_color(obj, color):
    DISPLAY_COLORS[obj.Name] = color
    view = getattr(obj, "ViewObject", None)
    if view is None:
        return
    set_view_common(view)
    try:
        view.ShapeColor = color
    except Exception:
        pass


def make_material(color):
    material = App.Material()
    material.DiffuseColor = color
    material.AmbientColor = (color[0] * 0.45, color[1] * 0.45, color[2] * 0.45, 1.0)
    material.EmissiveColor = (0.0, 0.0, 0.0, 1.0)
    material.SpecularColor = (0.35, 0.35, 0.35, 1.0)
    material.Transparency = 0.0
    return material


def set_view_common(view):
    try:
        view.Visibility = True
    except Exception:
        pass
    try:
        view.Transparency = 0
    except Exception:
        pass


def apply_face_colors(obj, colors):
    view = getattr(obj, "ViewObject", None)
    if view is None:
        return
    set_view_common(view)
    try:
        view.DiffuseColor = colors
    except Exception:
        pass
    try:
        view.ShapeAppearance = tuple(make_material(color) for color in colors)
    except Exception:
        pass


def export_visual_obj(obj):
    stl_path = os.path.join(VISUAL_DIR, f"{obj.Name}.stl")
    mesh = MeshPart.meshFromShape(
        Shape=obj.Shape,
        LinearDeflection=1.0,
        AngularDeflection=0.35,
        Relative=False,
    )
    mesh.write(stl_path)


def box(name, length, width, height, center, color=DEFAULT_COLOR):
    shape = Part.makeBox(length, width, height)
    shape.translate(App.Vector(center[0] - length / 2, center[1] - width / 2, center[2] - height / 2))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    apply_view_color(obj, color)
    export_visual_obj(obj)
    return obj


def cyl(name, radius, height, center, axis="z", color=DEFAULT_COLOR):
    shape = Part.makeCylinder(radius, height)
    if axis == "x":
        shape.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), 90)
    elif axis == "y":
        shape.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), 90)
    shape.translate(App.Vector(center[0], center[1], center[2]))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    apply_view_color(obj, color)
    export_visual_obj(obj)
    return obj


def sphere(name, radius, center, color=DEFAULT_COLOR):
    shape = Part.makeSphere(radius)
    shape.translate(App.Vector(center[0], center[1], center[2]))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    apply_view_color(obj, color)
    export_visual_obj(obj)
    return obj


def compound(name, objects):
    shape = Part.makeCompound([o.Shape for o in objects])
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    face_colors = []
    for source in objects:
        color = DISPLAY_COLORS.get(source.Name, DEFAULT_COLOR)
        face_colors.extend([color] * len(source.Shape.Faces))
    for o in objects:
        App.ActiveDocument.removeObject(o.Name)
    apply_view_color(obj, DEFAULT_COLOR)
    view = getattr(obj, "ViewObject", None)
    if view is not None and face_colors:
        apply_face_colors(obj, face_colors)
    return obj


def export_obj(obj, filename_base):
    stl_path = os.path.join(MESH_DIR, f"{filename_base}.stl")
    step_path = os.path.join(STEP_DIR, f"{filename_base}.step")
    mesh = MeshPart.meshFromShape(
        Shape=obj.Shape,
        LinearDeflection=1.0,
        AngularDeflection=0.35,
        Relative=False,
    )
    mesh.write(stl_path)
    obj.Shape.exportStep(step_path)


def add_frame_marker(name, pos, axis="z", color=JOINT_BLUE):
    return cyl(name, 6.0, 35.0, pos, axis=axis, color=color)


def build_geometry():
    doc = App.newDocument("face_screen_arm")

    base = compound(
        "base_link",
        [
            cyl("base_disc", P["base_radius"], P["base_height"], (0, 0, 0), "z"),
            cyl("mast", P["mast_radius"], P["mast_height"], (0, 0, P["base_height"]), "z"),
            box("base_front_ref", 90, 18, 18, (45, 0, P["base_height"] + 15)),
        ],
    )

    yaw = compound(
        "yaw_link",
        [
            cyl("yaw_bearing", P["joint_radius"], P["joint_width"], (0, -P["joint_width"] / 2, 0), "y", color=JOINT_BLUE),
            box("shoulder_yoke_l", 32, 18, 90, (0, -40, -15), color=JOINT_BLUE),
            box("shoulder_yoke_r", 32, 18, 90, (0, 40, -15), color=JOINT_BLUE),
            add_frame_marker("joint2_axis_hint", (0, -18, 0), "y"),
        ],
    )

    upper = compound(
        "upper_arm",
        [
            box("upper_bar", P["upper_len"], P["arm_width"], P["arm_height"], (P["upper_len"] / 2, 0, 0)),
            cyl("upper_shoulder_hub", P["joint_radius"], P["joint_width"], (0, -P["joint_width"] / 2, 0), "y", color=JOINT_BLUE),
            cyl("upper_elbow_hub", P["joint_radius"], P["joint_width"], (P["upper_len"], -P["joint_width"] / 2, 0), "y", color=JOINT_BLUE),
        ],
    )

    forearm = compound(
        "forearm",
        [
            box("forearm_bar", P["forearm_len"], P["arm_width"], P["arm_height"], (P["forearm_len"] / 2, 0, 0)),
            cyl("forearm_elbow_hub", P["joint_radius"], P["joint_width"], (0, -P["joint_width"] / 2, 0), "y", color=JOINT_BLUE),
            box("slider_outer_tube", 260, 68, 56, (P["forearm_len"] - 90, 0, 0)),
        ],
    )

    tube_len = P["tube_insert"] + P["tube_out"]
    tube_center = (P["tube_out"] - P["tube_insert"]) / 2
    wrist_x = P["tube_out"] + P["wrist_gap"]
    slider = compound(
        "telescopic_slider",
        [
            box("inner_tube", tube_len, 36, 32, (tube_center, 0, 0)),
            box("linear_rail_hint", tube_len, 8, 40, (tube_center, 25, 0)),
            box("telescopic_end_cap", 28, 48, 46, (wrist_x, 0, 0)),
            cyl("pan_mount_boss", 24, 18, (wrist_x, 0, P["pan_mount_z"] - 9), "z", color=JOINT_BLUE),
        ],
    )

    pan = compound(
        "screen_pan_link",
        [
            cyl("pan_base_disc", 45, 16, (0, 0, -8), "z", color=JOINT_BLUE),
            cyl("pan_rotor_cap", 34, 10, (0, 0, 8), "z", color=JOINT_BLUE),
            box("low_pitch_bridge", 112, 36, 18, (42, 0, 28)),
            box("pitch_side_plate_l", 24, 14, 34, (P["pan_pitch_x"], -39, P["pan_pitch_z"] - 8), color=JOINT_BLUE),
            box("pitch_side_plate_r", 24, 14, 34, (P["pan_pitch_x"], 39, P["pan_pitch_z"] - 8), color=JOINT_BLUE),
            cyl("pitch_axis_hint", 16, 78, (P["pan_pitch_x"], -39, P["pan_pitch_z"]), "y"),
        ],
    )

    screen = compound(
        "screen_pitch_link",
        [
            cyl("screen_pitch_hub", 18, 90, (0, -45, 0), "y", color=JOINT_BLUE),
            box("screen_plate", P["screen_thick"], P["screen_width"], P["screen_height"], (45, 0, P["screen_center_z"]), color=SCREEN_BLACK),
            box("screen_lower_mount", 42, 120, 96, (18, 0, P["screen_center_z"] / 2)),
            box("screen_back_mount", 36, 86, 96, (20, 0, P["screen_center_z"] / 2)),
            sphere("screen_center_marker", 8, (P["screen_center_x"], 0, P["screen_center_z"]), color=CENTER_RED),
        ],
    )

    App.ActiveDocument.recompute()
    for obj in (base, yaw, upper, forearm, slider, pan, screen):
        export_obj(obj, obj.Name)

    fcstd_path = os.path.join(OUT, "face_screen_arm.FCStd")
    step_all_path = os.path.join(OUT, "face_screen_arm_preview.step")
    doc.saveAs(fcstd_path)
    Part.export([base, yaw, upper, forearm, slider, pan, screen], step_all_path)
    return fcstd_path, step_all_path


def meters(mm):
    return mm / 1000.0


def mesh_tag(name):
    return f'<mesh filename="../meshes/{name}.stl" scale="0.001 0.001 0.001"/>'


def visual_mesh_tag(name):
    return f'<mesh filename="../visuals/{name}.stl" scale="0.001 0.001 0.001"/>'


def visual_tag(name, material):
    return f'<visual name="{name}"><geometry>{visual_mesh_tag(name)}</geometry><material name="{material}"/></visual>'


def write_urdf():
    sh = meters(P["shoulder_height"])
    l1 = meters(P["upper_len"])
    l2 = meters(P["forearm_len"])
    travel = meters(P["tube_travel"])
    wrist_x = meters(P["tube_out"] + P["wrist_gap"])
    pan_mount_z = meters(P["pan_mount_z"])
    pan_pitch_x = meters(P["pan_pitch_x"])
    pan_pitch_z = meters(P["pan_pitch_z"])
    screen_center_x = meters(P["screen_center_x"])
    screen_center_z = meters(P["screen_center_z"])

    urdf = f'''<?xml version="1.0"?>
<robot name="face_screen_support_arm">
  <material name="neutral_gray"><color rgba="0.78 0.78 0.76 1"/></material>
  <material name="joint_sky_blue"><color rgba="0.529 0.808 0.922 1"/></material>
  <material name="screen_black"><color rgba="0 0 0 1"/></material>
  <material name="center_red"><color rgba="1 0 0 1"/></material>

  <link name="base_link">
    {visual_tag("base_disc", "neutral_gray")}
    {visual_tag("mast", "neutral_gray")}
    {visual_tag("base_front_ref", "neutral_gray")}
    <collision><geometry>{mesh_tag("base_link")}</geometry></collision>
    <inertial><origin xyz="0 0 0.05"/><mass value="2.0"/><inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/></inertial>
  </link>

  <joint name="joint1_base_yaw" type="revolute">
    <origin xyz="0 0 {sh}" rpy="0 0 0"/>
    <parent link="base_link"/><child link="yaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.61799" upper="2.61799" effort="30" velocity="1.5"/>
  </joint>
  <link name="yaw_link">
    {visual_tag("yaw_bearing", "joint_sky_blue")}
    {visual_tag("shoulder_yoke_l", "joint_sky_blue")}
    {visual_tag("shoulder_yoke_r", "joint_sky_blue")}
    {visual_tag("joint2_axis_hint", "joint_sky_blue")}
    <collision><geometry>{mesh_tag("yaw_link")}</geometry></collision>
    <inertial><origin xyz="0 0 0"/><mass value="0.7"/><inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/></inertial>
  </link>

  <joint name="joint2_shoulder_pitch" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="yaw_link"/><child link="upper_arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="0" effort="35" velocity="1.2"/>
  </joint>
  <link name="upper_arm">
    {visual_tag("upper_bar", "neutral_gray")}
    {visual_tag("upper_shoulder_hub", "joint_sky_blue")}
    {visual_tag("upper_elbow_hub", "joint_sky_blue")}
    <collision><geometry>{mesh_tag("upper_arm")}</geometry></collision>
    <inertial><origin xyz="{l1 / 2} 0 0"/><mass value="0.8"/><inertia ixx="0.006" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/></inertial>
  </link>

  <joint name="joint3_elbow_pitch" type="revolute">
    <origin xyz="{l1} 0 0" rpy="0 0 0"/>
    <parent link="upper_arm"/><child link="forearm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0944" upper="2.61799" effort="30" velocity="1.2"/>
  </joint>
  <link name="forearm">
    {visual_tag("forearm_bar", "neutral_gray")}
    {visual_tag("forearm_elbow_hub", "joint_sky_blue")}
    {visual_tag("slider_outer_tube", "neutral_gray")}
    <collision><geometry>{mesh_tag("forearm")}</geometry></collision>
    <inertial><origin xyz="{l2 / 2} 0 0"/><mass value="0.7"/><inertia ixx="0.004" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.015"/></inertial>
  </link>

  <joint name="joint4_telescopic" type="prismatic">
    <origin xyz="{l2} 0 0" rpy="0 0 0"/>
    <parent link="forearm"/><child link="telescopic_slider"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="{travel}" effort="120" velocity="0.25"/>
  </joint>
  <link name="telescopic_slider">
    {visual_tag("inner_tube", "neutral_gray")}
    {visual_tag("linear_rail_hint", "neutral_gray")}
    {visual_tag("telescopic_end_cap", "neutral_gray")}
    {visual_tag("pan_mount_boss", "joint_sky_blue")}
    <collision><geometry>{mesh_tag("telescopic_slider")}</geometry></collision>
    <inertial><origin xyz="0 0 0.04"/><mass value="0.55"/><inertia ixx="0.003" ixy="0" ixz="0" iyy="0.012" iyz="0" izz="0.012"/></inertial>
  </link>

  <joint name="joint5_screen_pan" type="revolute">
    <origin xyz="{wrist_x} 0 {pan_mount_z}" rpy="0 0 0"/>
    <parent link="telescopic_slider"/><child link="screen_pan_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="8" velocity="2.0"/>
  </joint>
  <link name="screen_pan_link">
    {visual_tag("pan_base_disc", "joint_sky_blue")}
    {visual_tag("pan_rotor_cap", "joint_sky_blue")}
    {visual_tag("low_pitch_bridge", "neutral_gray")}
    {visual_tag("pitch_side_plate_l", "joint_sky_blue")}
    {visual_tag("pitch_side_plate_r", "joint_sky_blue")}
    {visual_tag("pitch_axis_hint", "joint_sky_blue")}
    <collision><geometry>{mesh_tag("screen_pan_link")}</geometry></collision>
    <inertial><origin xyz="0 0 0"/><mass value="0.25"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
  </link>

  <joint name="joint6_screen_pitch" type="revolute">
    <origin xyz="{pan_pitch_x} 0 {pan_pitch_z}" rpy="0 0 0"/>
    <parent link="screen_pan_link"/><child link="screen_pitch_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0472" upper="1.0472" effort="8" velocity="2.0"/>
  </joint>
  <link name="screen_pitch_link">
    {visual_tag("screen_pitch_hub", "joint_sky_blue")}
    {visual_tag("screen_plate", "screen_black")}
    {visual_tag("screen_lower_mount", "neutral_gray")}
    {visual_tag("screen_back_mount", "neutral_gray")}
    {visual_tag("screen_center_marker", "center_red")}
    <inertial><origin xyz="0.04 0 0"/><mass value="0.65"/><inertia ixx="0.006" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.008"/></inertial>
  </link>

  <joint name="screen_center_fixed" type="fixed">
    <origin xyz="{screen_center_x} 0 {screen_center_z}" rpy="0 0 0"/>
    <parent link="screen_pitch_link"/><child link="screen_center"/>
  </joint>
  <link name="screen_center"/>
</robot>
'''
    path = os.path.join(URDF_DIR, "face_screen_support_arm.urdf")
    with open(path, "w", encoding="utf-8") as f:
        f.write(urdf)
    return path


def main():
    ensure_dirs()
    fcstd, step_preview = build_geometry()
    urdf = write_urdf()
    print(f"FCStd={fcstd}")
    print(f"STEP_PREVIEW={step_preview}")
    print(f"URDF={urdf}")
    print(f"MESH_DIR={MESH_DIR}")


if __name__ == "__main__":
    main()
