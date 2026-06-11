"""Apply presentation colors to the FreeCAD arm model.

Run inside FreeCAD:
    exec(open(r"E:\\robotics\\final_project\\screen_arm\\scripts\\apply_face_screen_arm_colors_freecad.py").read())

The script colors the screen plate black and rotational joint hardware sky blue,
then overwrites generated/face_screen_arm.FCStd.
"""

import os

import FreeCAD as App

try:
    import FreeCADGui as Gui
except Exception:
    Gui = None


DEFAULT_COLOR = (0.78, 0.78, 0.76, 1.0)
JOINT_BLUE = (135.0 / 255.0, 206.0 / 255.0, 235.0 / 255.0, 1.0)
SCREEN_BLACK = (0.0, 0.0, 0.0, 1.0)
CENTER_RED = (1.0, 0.0, 0.0, 1.0)


FACE_COLOR_COUNTS = {
    "base_link": [
        (3, DEFAULT_COLOR),  # base_disc
        (3, DEFAULT_COLOR),  # mast
        (6, DEFAULT_COLOR),  # base_front_ref
    ],
    "yaw_link": [
        (3, JOINT_BLUE),  # yaw_bearing
        (6, JOINT_BLUE),  # shoulder_yoke_l
        (6, JOINT_BLUE),  # shoulder_yoke_r
        (3, JOINT_BLUE),  # joint2_axis_hint
    ],
    "upper_arm": [
        (6, DEFAULT_COLOR),  # upper_bar
        (3, JOINT_BLUE),  # upper_shoulder_hub
        (3, JOINT_BLUE),  # upper_elbow_hub
    ],
    "forearm": [
        (6, DEFAULT_COLOR),  # forearm_bar
        (3, JOINT_BLUE),  # forearm_elbow_hub
        (6, DEFAULT_COLOR),  # slider_outer_tube
    ],
    "telescopic_slider": [
        (6, DEFAULT_COLOR),  # inner_tube
        (6, DEFAULT_COLOR),  # linear_rail_hint
        (6, DEFAULT_COLOR),  # telescopic_end_cap
        (3, JOINT_BLUE),  # pan_mount_boss
    ],
    "screen_pan_link": [
        (3, JOINT_BLUE),  # pan_base_disc
        (3, JOINT_BLUE),  # pan_rotor_cap
        (6, DEFAULT_COLOR),  # low_pitch_bridge
        (6, JOINT_BLUE),  # pitch_side_plate_l
        (6, JOINT_BLUE),  # pitch_side_plate_r
        (3, JOINT_BLUE),  # pitch_axis_hint
    ],
    "screen_pitch_link": [
        (3, JOINT_BLUE),  # screen_pitch_hub
        (6, SCREEN_BLACK),  # screen_plate
        (6, DEFAULT_COLOR),  # screen_lower_mount
        (6, DEFAULT_COLOR),  # screen_back_mount
        (1, CENTER_RED),  # screen_center_marker
    ],
}


def project_root():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(here) if os.path.basename(here).lower() == "scripts" else here


def get_doc():
    if App.ActiveDocument is not None:
        return App.ActiveDocument

    path = os.path.join(project_root(), "generated", "face_screen_arm.FCStd")
    return App.openDocument(path)


def expanded_colors(items):
    colors = []
    for count, color in items:
        colors.extend([color] * count)
    return colors


def make_material(color):
    material = App.Material()
    material.DiffuseColor = color
    material.AmbientColor = (color[0] * 0.45, color[1] * 0.45, color[2] * 0.45, 1.0)
    material.EmissiveColor = (0.0, 0.0, 0.0, 1.0)
    material.SpecularColor = (0.35, 0.35, 0.35, 1.0)
    material.Transparency = 0.0
    return material


def apply_colors():
    doc = get_doc()
    for name, color_counts in FACE_COLOR_COUNTS.items():
        obj = doc.getObject(name)
        if obj is None:
            raise RuntimeError("Missing object in FreeCAD document: {}".format(name))

        colors = expanded_colors(color_counts)
        if len(colors) != len(obj.Shape.Faces):
            raise RuntimeError(
                "{} has {} faces, but {} colors were prepared".format(
                    name, len(obj.Shape.Faces), len(colors)
                )
            )

        view = getattr(obj, "ViewObject", None)
        if view is not None:
            view.Visibility = True
            view.Transparency = 0
            view.ShapeColor = DEFAULT_COLOR
            try:
                view.DiffuseColor = colors
            except Exception:
                pass
            try:
                view.ShapeAppearance = tuple(make_material(color) for color in colors)
            except Exception:
                pass

    doc.recompute()

    output = os.path.join(project_root(), "generated", "face_screen_arm.FCStd")
    if os.path.abspath(getattr(doc, "FileName", "")) == os.path.abspath(output):
        doc.save()
    else:
        doc.saveAs(output)
    print("Saved colored FreeCAD model:", output)

    if Gui is not None and hasattr(Gui, "SendMsgToActiveView"):
        try:
            Gui.SendMsgToActiveView("ViewFit")
        except Exception:
            pass


apply_colors()
