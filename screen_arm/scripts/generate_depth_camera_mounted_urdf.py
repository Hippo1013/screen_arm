import os
import xml.etree.ElementTree as ET


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR) if os.path.basename(SCRIPT_DIR).lower() == "scripts" else SCRIPT_DIR
URDF_DIR = os.path.join(ROOT, "generated", "urdf")

SOURCE_URDF = os.path.join(URDF_DIR, "face_screen_support_arm.urdf")
OUTPUT_URDF = os.path.join(URDF_DIR, "face_screen_support_arm_depth_camera.urdf")

# Dimensions are in meters here. They match generate_face_screen_arm.py and
# generate_depth_camera.py after the STL meshes are scaled by 0.001 in URDF.
MAST_RADIUS = 0.038

# The camera mesh has local X from -50 mm to 50 mm, local Y from -10 mm to
# 10 mm, and local Z from 0 mm to 25.2 mm including the lens layer. Rotate it
# so local +Z, the lens normal, points to world +X. Then local Z=0 is the
# camera bottom/back plane and is placed on the mast front tangent plane.
CAMERA_ORIGIN_X = MAST_RADIUS
CAMERA_ORIGIN_Y = 0.0
CAMERA_ORIGIN_Z = 0.060
CAMERA_RPY = "1.570796 0 1.570796"


def indent(element, level=0):
    space = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = space + "  "
        for child in element:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = space
    if level and (not element.tail or not element.tail.strip()):
        element.tail = space


def add_material_if_missing(root, name, rgba):
    for material in root.findall("material"):
        if material.attrib.get("name") == name:
            return
    material = ET.Element("material", {"name": name})
    ET.SubElement(material, "color", {"rgba": rgba})
    root.insert(0, material)


def make_visual(name, mesh_filename, material_name):
    visual = ET.Element("visual", {"name": name})
    ET.SubElement(
        visual,
        "origin",
        {
            "xyz": f"{CAMERA_ORIGIN_X:.3f} {CAMERA_ORIGIN_Y:.3f} {CAMERA_ORIGIN_Z:.3f}",
            "rpy": CAMERA_RPY,
        },
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": mesh_filename, "scale": "0.001 0.001 0.001"})
    ET.SubElement(visual, "material", {"name": material_name})
    return visual


def replace_base_front_ref(root):
    base_link = root.find("./link[@name='base_link']")
    if base_link is None:
        raise RuntimeError("base_link was not found")

    for visual in list(base_link.findall("visual")):
        if visual.attrib.get("name") == "base_front_ref":
            base_link.remove(visual)

    base_link.insert(2, make_visual("depth_camera_body", "../visuals/depth_camera_body.stl", "depth_camera_body"))
    base_link.insert(3, make_visual("depth_camera_lens", "../visuals/depth_camera_lens.stl", "depth_camera_lens"))


def main():
    tree = ET.parse(SOURCE_URDF)
    root = tree.getroot()
    root.set("name", "face_screen_support_arm_depth_camera")

    add_material_if_missing(root, "depth_camera_body", "0.30 0.31 0.33 1")
    add_material_if_missing(root, "depth_camera_lens", "0 0 0 1")
    replace_base_front_ref(root)

    indent(root)
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"URDF={OUTPUT_URDF}")
    print(f"CAMERA_ORIGIN={CAMERA_ORIGIN_X:.3f} {CAMERA_ORIGIN_Y:.3f} {CAMERA_ORIGIN_Z:.3f}")
    print(f"CAMERA_RPY={CAMERA_RPY}")


if __name__ == "__main__":
    main()
