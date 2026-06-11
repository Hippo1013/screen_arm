"""Pose the face screen support arm in FreeCAD.

Run from FreeCAD with:
    Macro -> Macros... -> Execute

The script expects generated/face_screen_arm.FCStd to be open. If no document is
open, it tries to open that file relative to this script.
"""

import math
import os

import FreeCAD as App

try:
    import FreeCADGui as Gui
except Exception:
    Gui = None


# Joint values for the default viewing pose.
# Angles are in degrees; the telescopic extension is in millimeters.
J1_BASE_YAW_DEG = 0.0
J2_SHOULDER_PITCH_DEG = -120.0
J3_ELBOW_PITCH_DEG = 120.0
J4_TELESCOPIC_MM = 30.0
J5_SCREEN_PAN_DEG = 0.0
J6_SCREEN_PITCH_DEG = 0.0


def trans(x, y, z):
    return App.Placement(App.Vector(x, y, z), App.Rotation())


def rot(axis, deg):
    return App.Placement(App.Vector(0, 0, 0), App.Rotation(App.Vector(*axis), deg))


def get_doc():
    if App.ActiveDocument is not None:
        return App.ActiveDocument

    here = os.path.dirname(os.path.abspath(__file__))
    fcstd = os.path.abspath(os.path.join(here, "..", "generated", "face_screen_arm.FCStd"))
    return App.openDocument(fcstd)


def obj(doc, name):
    item = doc.getObject(name)
    if item is None:
        raise RuntimeError("Missing object in FreeCAD document: {}".format(name))
    return item


def apply_pose():
    doc = get_doc()

    base = obj(doc, "base_link")
    yaw = obj(doc, "yaw_link")
    upper = obj(doc, "upper_arm")
    forearm = obj(doc, "forearm")
    slider = obj(doc, "telescopic_slider")
    pan = obj(doc, "screen_pan_link")
    screen = obj(doc, "screen_pitch_link")

    base.Placement = App.Placement()

    yaw_pose = base.Placement * trans(0, 0, 160) * rot((0, 0, 1), J1_BASE_YAW_DEG)
    upper_pose = yaw_pose * rot((0, 1, 0), J2_SHOULDER_PITCH_DEG)
    forearm_pose = upper_pose * trans(280, 0, 0) * rot((0, 1, 0), J3_ELBOW_PITCH_DEG)
    slider_pose = forearm_pose * trans(240 + J4_TELESCOPIC_MM, 0, 0)
    pan_pose = slider_pose * trans(85, 0, 24) * rot((0, 0, 1), J5_SCREEN_PAN_DEG)
    screen_pose = pan_pose * trans(70, 0, 42) * rot((0, 1, 0), J6_SCREEN_PITCH_DEG)

    yaw.Placement = yaw_pose
    upper.Placement = upper_pose
    forearm.Placement = forearm_pose
    slider.Placement = slider_pose
    pan.Placement = pan_pose
    screen.Placement = screen_pose

    doc.recompute()
    if Gui is not None and hasattr(Gui, "SendMsgToActiveView"):
        try:
            Gui.SendMsgToActiveView("ViewFit")
        except Exception:
            pass


apply_pose()
