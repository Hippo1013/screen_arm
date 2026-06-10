import os

import FreeCAD as App
import MeshPart
import Part


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR) if os.path.basename(SCRIPT_DIR).lower() == "scripts" else SCRIPT_DIR
OUT = os.path.join(ROOT, "generated")
MESH_DIR = os.path.join(OUT, "meshes")
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
    "tube_travel": 450.0,
    "pan_mount_z": 24.0,
    "pan_pitch_x": 50.0,
    "pan_pitch_z": 42.0,
    "screen_center_z": 72.0,
    "arm_width": 52.0,
    "arm_height": 42.0,
    "joint_radius": 34.0,
    "joint_width": 62.0,
    "wrist_gap": 0.0,
    "screen_width": 240.0,
    "screen_height": 160.0,
    "screen_thick": 14.0,
}


def ensure_dirs():
    for path in (OUT, MESH_DIR, STEP_DIR, URDF_DIR):
        os.makedirs(path, exist_ok=True)


def box(name, length, width, height, center):
    shape = Part.makeBox(length, width, height)
    shape.translate(App.Vector(center[0] - length / 2, center[1] - width / 2, center[2] - height / 2))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    return obj


def cyl(name, radius, height, center, axis="z"):
    shape = Part.makeCylinder(radius, height)
    if axis == "x":
        shape.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), 90)
    elif axis == "y":
        shape.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), 90)
    shape.translate(App.Vector(center[0], center[1], center[2]))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    return obj


def compound(name, objects):
    shape = Part.makeCompound([o.Shape for o in objects])
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    for o in objects:
        App.ActiveDocument.removeObject(o.Name)
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


def add_frame_marker(name, pos, axis="z"):
    return cyl(name, 6.0, 35.0, pos, axis=axis)


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
            cyl("yaw_bearing", P["joint_radius"], P["joint_width"], (0, -P["joint_width"] / 2, 0), "y"),
            box("shoulder_yoke_l", 32, 18, 90, (0, -40, -15)),
            box("shoulder_yoke_r", 32, 18, 90, (0, 40, -15)),
            add_frame_marker("joint2_axis_hint", (0, -18, 0), "y"),
        ],
    )

    upper = compound(
        "upper_arm",
        [
            box("upper_bar", P["upper_len"], P["arm_width"], P["arm_height"], (P["upper_len"] / 2, 0, 0)),
            cyl("upper_shoulder_hub", P["joint_radius"], P["joint_width"], (0, -P["joint_width"] / 2, 0), "y"),
            cyl("upper_elbow_hub", P["joint_radius"], P["joint_width"], (P["upper_len"], -P["joint_width"] / 2, 0), "y"),
        ],
    )

    forearm = compound(
        "forearm",
        [
            box("forearm_bar", P["forearm_len"], P["arm_width"], P["arm_height"], (P["forearm_len"] / 2, 0, 0)),
            cyl("forearm_elbow_hub", P["joint_radius"], P["joint_width"], (0, -P["joint_width"] / 2, 0), "y"),
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
            cyl("pan_mount_boss", 24, 18, (wrist_x, 0, P["pan_mount_z"] - 9), "z"),
        ],
    )

    pan = compound(
        "screen_pan_link",
        [
            cyl("pan_base_disc", 45, 16, (0, 0, -8), "z"),
            cyl("pan_rotor_cap", 34, 10, (0, 0, 8), "z"),
            box("low_pitch_bridge", 82, 36, 18, (28, 0, 28)),
            box("pitch_side_plate_l", 24, 14, 34, (P["pan_pitch_x"], -39, P["pan_pitch_z"] - 8)),
            box("pitch_side_plate_r", 24, 14, 34, (P["pan_pitch_x"], 39, P["pan_pitch_z"] - 8)),
            cyl("pitch_axis_hint", 16, 78, (P["pan_pitch_x"], -39, P["pan_pitch_z"]), "y"),
        ],
    )

    screen = compound(
        "screen_pitch_link",
        [
            cyl("screen_pitch_hub", 18, 90, (0, -45, 0), "y"),
            box("screen_plate", P["screen_thick"], P["screen_width"], P["screen_height"], (45, 0, P["screen_center_z"])),
            box("screen_lower_mount", 42, 86, 34, (18, 0, 12)),
            box("screen_back_mount", 34, 70, 58, (20, 0, 38)),
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


def write_urdf():
    sh = meters(P["shoulder_height"])
    l1 = meters(P["upper_len"])
    l2 = meters(P["forearm_len"])
    travel = meters(P["tube_travel"])
    wrist_x = meters(P["tube_out"] + P["wrist_gap"])
    pan_mount_z = meters(P["pan_mount_z"])
    pan_pitch_x = meters(P["pan_pitch_x"])
    pan_pitch_z = meters(P["pan_pitch_z"])
    screen_center_z = meters(P["screen_center_z"])

    urdf = f'''<?xml version="1.0"?>
<robot name="face_screen_support_arm">
  <link name="base_link">
    <visual><geometry>{mesh_tag("base_link")}</geometry></visual>
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
    <visual><geometry>{mesh_tag("yaw_link")}</geometry></visual>
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
    <visual><geometry>{mesh_tag("upper_arm")}</geometry></visual>
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
    <visual><geometry>{mesh_tag("forearm")}</geometry></visual>
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
    <visual><geometry>{mesh_tag("telescopic_slider")}</geometry></visual>
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
    <visual><geometry>{mesh_tag("screen_pan_link")}</geometry></visual>
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
    <visual><geometry>{mesh_tag("screen_pitch_link")}</geometry></visual>
    <inertial><origin xyz="0.04 0 0"/><mass value="0.65"/><inertia ixx="0.006" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.008"/></inertial>
  </link>

  <joint name="screen_center_fixed" type="fixed">
    <origin xyz="0.052 0 {screen_center_z}" rpy="0 0 0"/>
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
