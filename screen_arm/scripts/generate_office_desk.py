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
    "desk_width": 1400.0,
    "desk_depth": 750.0,
    "desk_height": 740.0,
    "top_thickness": 40.0,
    "leg_size": 50.0,
    "leg_inset_x": 90.0,
    "leg_inset_y": 80.0,
    "modesty_height": 240.0,
    "modesty_thickness": 18.0,
    "grommet_radius": 35.0,
    "foot_radius": 42.0,
    "foot_height": 10.0,
}

WOOD = (0.62, 0.42, 0.24, 1.0)
DARK_WOOD = (0.38, 0.25, 0.15, 1.0)
METAL = (0.30, 0.31, 0.32, 1.0)
BLACK = (0.03, 0.03, 0.035, 1.0)

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
    material.SpecularColor = (0.25, 0.25, 0.25, 1.0)
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


def box(name, length, width, height, center, color):
    shape = Part.makeBox(length, width, height)
    shape.translate(App.Vector(center[0] - length / 2, center[1] - width / 2, center[2] - height / 2))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    apply_view_color(obj, color)
    return obj


def cyl(name, radius, height, center, color, axis="z"):
    shape = Part.makeCylinder(radius, height)
    if axis == "x":
        shape.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), 90)
    elif axis == "y":
        shape.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), 90)
    shape.translate(App.Vector(center[0], center[1], center[2]))
    obj = App.ActiveDocument.addObject("Part::Feature", name)
    obj.Shape = shape
    apply_view_color(obj, color)
    return obj


def rounded_box(name, length, width, height, center, radius, color):
    base = Part.makeBox(length - 2 * radius, width, height)
    base.translate(App.Vector(center[0] - (length - 2 * radius) / 2, center[1] - width / 2, center[2] - height / 2))
    side = Part.makeBox(length, width - 2 * radius, height)
    side.translate(App.Vector(center[0] - length / 2, center[1] - (width - 2 * radius) / 2, center[2] - height / 2))
    corners = []
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
            corners.append(corner)
    shape = base.fuse(side)
    for corner in corners:
        shape = shape.fuse(corner)
    shape = shape.removeSplitter()
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
        color = DISPLAY_COLORS.get(source.Name, WOOD)
        face_colors.extend([color] * len(source.Shape.Faces))
    for source in objects:
        App.ActiveDocument.removeObject(source.Name)
    apply_view_color(obj, WOOD)
    if face_colors:
        apply_face_colors(obj, face_colors)
    return obj


def export_mesh(obj, path, linear_deflection=1.0, angular_deflection=0.35):
    mesh = MeshPart.meshFromShape(
        Shape=obj.Shape,
        LinearDeflection=linear_deflection,
        AngularDeflection=angular_deflection,
        Relative=False,
    )
    mesh.write(path)


def export_obj(obj):
    export_mesh(obj, os.path.join(MESH_DIR, "office_desk.stl"))
    export_mesh(obj, os.path.join(VISUAL_DIR, "office_desk.stl"))
    obj.Shape.exportStep(os.path.join(STEP_DIR, "office_desk.step"))


def make_desk():
    width = P["desk_width"]
    depth = P["desk_depth"]
    height = P["desk_height"]
    top_thickness = P["top_thickness"]
    leg_size = P["leg_size"]
    leg_height = height - top_thickness
    top_center_z = height - top_thickness / 2

    objects = []
    top = rounded_box("office_desk_top", depth, width, top_thickness, (0, 0, top_center_z), 35.0, WOOD)
    objects.append(top)

    # Thin dark edge band makes the tabletop easier to read in CAD without changing collision geometry much.
    edge_front = box("office_desk_front_edge", 12.0, width - 70.0, 32.0, (depth / 2 + 1.0, 0, top_center_z), DARK_WOOD)
    edge_back = box("office_desk_back_edge", 12.0, width - 70.0, 32.0, (-depth / 2 - 1.0, 0, top_center_z), DARK_WOOD)
    objects.extend([edge_front, edge_back])

    leg_x = depth / 2 - P["leg_inset_x"]
    leg_y = width / 2 - P["leg_inset_y"]
    for ix, x in enumerate((-leg_x, leg_x), start=1):
        for iy, y in enumerate((-leg_y, leg_y), start=1):
            leg_name = "office_desk_leg_%d%d" % (ix, iy)
            foot_name = "office_desk_foot_%d%d" % (ix, iy)
            objects.append(box(leg_name, leg_size, leg_size, leg_height, (x, y, leg_height / 2), METAL))
            objects.append(cyl(foot_name, P["foot_radius"], P["foot_height"], (x, y, 0), BLACK))

    modesty_z = height - top_thickness - P["modesty_height"] / 2 - 45.0
    modesty_x = -depth / 2 + P["modesty_thickness"] / 2 + 45.0
    objects.append(
        box(
            "office_desk_modesty_panel",
            P["modesty_thickness"],
            width - 220.0,
            P["modesty_height"],
            (modesty_x, 0, modesty_z),
            DARK_WOOD,
        )
    )

    grommet_x = -depth / 2 + 120.0
    grommet_y = width / 2 - 210.0
    objects.append(cyl("office_desk_cable_grommet", P["grommet_radius"], 5.0, (grommet_x, grommet_y, height + 1.0), BLACK))

    return compound("office_desk", objects)


def main():
    ensure_dirs()
    doc = App.newDocument("office_desk")
    desk = make_desk()
    doc.recompute()
    export_obj(desk)
    doc.saveAs(os.path.join(OUT, "office_desk.FCStd"))


if __name__ == "__main__":
    main()
