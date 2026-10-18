#!/usr/bin/env python3
"""Convert a RealmForge XR world export to a Gazebo SDF static-obstacle world.

Coordinate contract: RealmForge (x, y-up, z) -> Gazebo ENU (x, -z, y).
RealmForge yaw about +Y -> Gazebo yaw about +Z with the same sign.
This intentionally exports primitive collision boxes, NOT scanned meshes,
navigation grids, robot flight plans, or verified sensor models.
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

BLOCKING = {"wall", "building", "pallet", "rack"}


def convert(data):
    if data.get("format") != "realmforge-world" or data.get("version") != 1:
        raise ValueError("Expected realmforge-world version 1")
    objects = data.get("objects")
    if not isinstance(objects, list) or len(objects) > 2000:
        raise ValueError("Expected objects array of at most 2000 entries")
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="realmforge")
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    ground = ET.SubElement(world, "model", name="ground")
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", name="ground_link")
    ET.SubElement(link, "pose").text = "0 0 -0.05 0 0 0"
    for label in ("collision", "visual"):
        elem = ET.SubElement(link, label, name=label)
        box = ET.SubElement(ET.SubElement(elem, "geometry"), "box")
        ET.SubElement(box, "size").text = "80 80 0.1"
    count = 0
    for index, obj in enumerate(objects):
        if not isinstance(obj, dict):
            raise ValueError(f"Object {index} is not an object")
        kind = obj.get("kind")
        size = obj.get("size")
        pos = obj.get("position")
        yaw = obj.get("rotationY")
        if (not isinstance(size, list) or len(size) != 3 or
            not isinstance(pos, list) or len(pos) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in size + pos) or
            any(v <= 0 for v in size) or
            not isinstance(yaw, (int, float)) or not math.isfinite(yaw)):
            raise ValueError(f"Invalid transform for object {index}")
        solid = obj.get("solid")
        if solid is not None and not isinstance(solid, bool):
            raise ValueError(f"Invalid solid flag for object {index}")
        blocking = solid if solid is not None else (
            size[1] > 0.3 if kind == "platform" else kind in BLOCKING
        )
        if not blocking:
            continue
        count += 1
        model = ET.SubElement(world, "model", name=f"obstacle_{index}")
        ET.SubElement(model, "static").text = "true"
        link = ET.SubElement(model, "link", name="body")
        # Source position is base center; Gazebo boxes are center-origin.
        ET.SubElement(link, "pose").text = (
            f"{pos[0]} {-pos[2]} {pos[1] + size[1] / 2} 0 0 {yaw}"
        )
        for label in ("collision", "visual"):
            elem = ET.SubElement(link, label, name=label)
            box = ET.SubElement(ET.SubElement(elem, "geometry"), "box")
            ET.SubElement(box, "size").text = f"{size[0]} {size[2]} {size[1]}"
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode", xml_declaration=True), count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="RealmForge exported .json")
    parser.add_argument("output", type=Path, help="Gazebo .sdf output")
    args = parser.parse_args()
    sdf, count = convert(json.loads(args.input.read_text(encoding="utf-8")))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(sdf + "\n", encoding="utf-8")
    print(f"Exported {count} collision obstacles to {args.output}")


if __name__ == "__main__":
    main()
