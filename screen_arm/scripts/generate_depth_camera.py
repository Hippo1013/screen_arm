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


P = {
    "camera_length": 100.0,
    "camera_width": 20.0,
    "camera_height": 24.0,
    "corner_radius": 9.0,
    "lens_length": 70.0,
    "lens_width": 13.0,
    "lens_thickness": 1.2,
    "lens_corner_radius": 4.5,
}

BODY_COLOR = (0.30, 0.31, 0.33, 1.0)
LENS_BLACK = (0.0, 0.0, 0.0, 1.0)
DISPLAY_COLORS = {}


def ensure_dirs():
    for path in (OUT, MESH_DIR, VISUAL_DIR, STEP_DIR):
        os.makedirs(path, exist_ok=True)


def set_view_common(view):
    try:
        view.Visibility = True
    except Exception:
        pass
    try:
        view.Transparency = 0
    except Exception:
        pass


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


def rounded_box_shape(length, width, height, radius, center):
    core_x = Part.makeBox(length - 2 * radius, width, height)
    core_x.translate(App.Vector(center[0] - (length - 2 * radius) / 2, center[1] - width / 2, center[2] - height / 2))
    core_y = Part.makeBox(length, width - 2 * radius, height)
    core_y.translate(App.Vector(center[0] - length / 2, center[1] - (width - 2 * radius) / 2, center[2] - height / 2))

    shape = core_x.fuse(core_y)
    for sx in (-1, 1):
        for sy in (-1, 1):
            corner = Part.makeCylinder(radius, height)
            corner.translate(
                App.Vector(
                    center[0] + sx * (length / 2 - radius),
                    center[1] + sy * (width / 2 - radius),
                    center[2] - height / 2,
                )
            )
            shape = shape.fuse(corner)
    return shape.removeSplitter()


def add_part(name, shape, color):
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    apply_view_color(obj, color)
    return obj


def compound(name, objects):
    shape = Part.makeCompound([obj.Shape for obj in objects])
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape

    face_colors = []
    for source in objects:
        color = DISPLAY_COLORS.get(source.Name, BODY_COLOR)
        face_colors.extend([color] * len(source.Shape.Faces))
    for source in objects:
        App.ActiveDocument.removeObject(source.Name)
    apply_view_color(obj, BODY_COLOR)
    apply_face_colors(obj, face_colors)
    return obj


def export_mesh(obj, path):
    mesh = MeshPart.meshFromShape(
        Shape=obj.Shape,
        LinearDeflection=0.5,
        AngularDeflection=0.25,
        Relative=False,
    )
    mesh.write(path)


def make_depth_camera():
    body = add_part(
        "depth_camera_body",
        rounded_box_shape(
            P["camera_length"],
            P["camera_width"],
            P["camera_height"],
            P["corner_radius"],
            (0, 0, P["camera_height"] / 2),
        ),
        BODY_COLOR,
    )

    lens = add_part(
        "depth_camera_lens",
        rounded_box_shape(
            P["lens_length"],
            P["lens_width"],
            P["lens_thickness"],
            P["lens_corner_radius"],
            (0, 0, P["camera_height"] + P["lens_thickness"] / 2),
        ),
        LENS_BLACK,
    )

    export_mesh(body, os.path.join(VISUAL_DIR, "depth_camera_body.stl"))
    export_mesh(lens, os.path.join(VISUAL_DIR, "depth_camera_lens.stl"))
    return compound("depth_camera", [body, lens])


def main():
    ensure_dirs()
    doc = App.newDocument("depth_camera")
    camera = make_depth_camera()
    doc.recompute()

    export_mesh(camera, os.path.join(MESH_DIR, "depth_camera.stl"))
    export_mesh(camera, os.path.join(VISUAL_DIR, "depth_camera.stl"))
    camera.Shape.exportStep(os.path.join(STEP_DIR, "depth_camera.step"))
    doc.saveAs(os.path.join(OUT, "depth_camera.FCStd"))


if __name__ == "__main__":
    main()
