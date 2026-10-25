#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


DOOR_PATCH_SIZE = (1.40, 0.26, 0.01)
DOOR_LINE_SIZE = (1.22, 0.03, 0.005)
DEAD_END_PATCH_SIZE = (0.95, 0.95, 0.01)
DEAD_END_CROSS_SIZE = (0.83, 0.05, 0.005)
WATER_PATCH_SIZE = (0.80, 0.55, 0.01)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Append semantic_marker_* models to an SDF world from point JSON files."
    )
    parser.add_argument("--world", type=Path, required=True, help="Input .world file")
    parser.add_argument("--output", type=Path, required=True, help="Output .world file")
    parser.add_argument("--door-json", type=Path, help="JSON produced by pick_world_points.py for door points")
    parser.add_argument(
        "--dead-end-json",
        type=Path,
        help="JSON produced by pick_world_points.py for dead_end points",
    )
    parser.add_argument("--water-json", type=Path, help="JSON produced by pick_world_points.py for water points")
    parser.add_argument(
        "--clear-existing",
        action="store_true",
        help="Remove existing semantic_marker_* models before appending the new ones",
    )
    parser.add_argument(
        "--infer-door-yaw",
        action="store_true",
        help="Infer door yaw from nearby wall boxes when yaw is not provided in JSON",
    )
    return parser.parse_args()


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        for child in elem:
            indent_xml(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = indent
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


def load_points(path: Path | None) -> list[dict]:
    if path is None:
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    points = data.get("points", [])
    if not isinstance(points, list):
        raise RuntimeError(f"Invalid points payload in {path}")
    cleaned = []
    for point in points:
        cleaned.append(
            {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "yaw": float(point.get("yaw", 0.0)),
            }
        )
    return cleaned


def normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def world_transform(base_pose: list[float], local_pose: list[float]) -> tuple[float, float, float]:
    bx, by, _, _, _, byaw = base_pose
    lx, ly, _, _, _, lyaw = local_pose
    x = bx + math.cos(byaw) * lx - math.sin(byaw) * ly
    y = by + math.sin(byaw) * lx + math.cos(byaw) * ly
    return x, y, normalize_angle(byaw + lyaw)


def parse_pose(text: str | None) -> list[float]:
    if not text:
        return [0.0] * 6
    values = [float(v) for v in text.split()]
    while len(values) < 6:
        values.append(0.0)
    return values[:6]


def load_wall_boxes(world: ET.Element) -> list[dict]:
    boxes: list[dict] = []
    for model in world.findall("model"):
        model_name = model.get("name", "")
        if model_name == "ground_plane" or model_name.startswith("semantic_marker_"):
            continue
        model_pose = parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            link_pose = parse_pose(link.findtext("pose"))
            link_x, link_y, link_yaw = world_transform(model_pose, link_pose)
            link_world_pose = [link_x, link_y, 0.0, 0.0, 0.0, link_yaw]
            for collision in link.findall("collision"):
                box_size_text = collision.findtext("geometry/box/size")
                if not box_size_text:
                    continue
                size_values = [float(v) for v in box_size_text.split()]
                if len(size_values) < 2:
                    continue
                collision_pose = parse_pose(collision.findtext("pose"))
                cx, cy, cyaw = world_transform(link_world_pose, collision_pose)
                sx, sy = size_values[0], size_values[1]
                if max(sx, sy) < 0.5:
                    continue
                major_yaw = cyaw if sx >= sy else normalize_angle(cyaw + math.pi * 0.5)
                boxes.append(
                    {
                        "x": cx,
                        "y": cy,
                        "sx": sx,
                        "sy": sy,
                        "yaw": cyaw,
                        "major_yaw": major_yaw,
                    }
                )
    return boxes


def point_to_oriented_box_distance(px: float, py: float, box: dict) -> float:
    dx = px - box["x"]
    dy = py - box["y"]
    c = math.cos(box["yaw"])
    s = math.sin(box["yaw"])
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    hx = box["sx"] * 0.5
    hy = box["sy"] * 0.5
    dx_out = max(abs(local_x) - hx, 0.0)
    dy_out = max(abs(local_y) - hy, 0.0)
    return math.hypot(dx_out, dy_out)


def infer_door_yaw(x: float, y: float, wall_boxes: list[dict]) -> float:
    if not wall_boxes:
        return 0.0
    best = min(
        wall_boxes,
        key=lambda box: (point_to_oriented_box_distance(x, y, box), abs(max(box["sx"], box["sy"]))),
    )
    return float(best["major_yaw"])


def add_text_element(parent: ET.Element, tag: str, text: str) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = text
    return elem


def fmt_pose(x: float, y: float, yaw: float) -> str:
    return f"{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}"


def fmt_size(size: tuple[float, float, float]) -> str:
    return f"{size[0]:.2f} {size[1]:.2f} {size[2]:.3f}"


def add_box_visual(
    link: ET.Element,
    *,
    name: str,
    pose: str,
    size: tuple[float, float, float],
    ambient: str,
    diffuse: str,
    specular: str | None = None,
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    add_text_element(visual, "pose", pose)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    add_text_element(box, "size", fmt_size(size))
    material = ET.SubElement(visual, "material")
    add_text_element(material, "ambient", ambient)
    add_text_element(material, "diffuse", diffuse)
    if specular is not None:
        add_text_element(material, "specular", specular)


def build_door_marker(name: str, x: float, y: float, yaw: float) -> ET.Element:
    model = ET.Element("model", {"name": name})
    add_text_element(model, "static", "true")
    add_text_element(model, "pose", fmt_pose(x, y, yaw))
    link = ET.SubElement(model, "link", {"name": "link"})
    add_box_visual(
        link,
        name="door_patch",
        pose="0 0 0.005 0 0 0",
        size=DOOR_PATCH_SIZE,
        ambient="0.96 0.96 0.96 1",
        diffuse="0.96 0.96 0.96 1",
    )
    add_box_visual(
        link,
        name="door_center_line",
        pose="0 0 0.011 0 0 0",
        size=DOOR_LINE_SIZE,
        ambient="0.15 0.15 0.15 1",
        diffuse="0.15 0.15 0.15 1",
    )
    return model


def build_dead_end_marker(name: str, x: float, y: float, yaw: float) -> ET.Element:
    model = ET.Element("model", {"name": name})
    add_text_element(model, "static", "true")
    add_text_element(model, "pose", fmt_pose(x, y, yaw))
    link = ET.SubElement(model, "link", {"name": "link"})
    add_box_visual(
        link,
        name="dead_end_patch",
        pose="0 0 0.005 0 0 0",
        size=DEAD_END_PATCH_SIZE,
        ambient="0.85 0.10 0.10 1",
        diffuse="0.85 0.10 0.10 1",
    )
    add_box_visual(
        link,
        name="cross_bar_a",
        pose="0 0 0.011 0 0 0.78",
        size=DEAD_END_CROSS_SIZE,
        ambient="0.98 0.98 0.98 1",
        diffuse="0.98 0.98 0.98 1",
    )
    add_box_visual(
        link,
        name="cross_bar_b",
        pose="0 0 0.011 0 0 -0.78",
        size=DEAD_END_CROSS_SIZE,
        ambient="0.98 0.98 0.98 1",
        diffuse="0.98 0.98 0.98 1",
    )
    return model


def build_water_marker(name: str, x: float, y: float, yaw: float) -> ET.Element:
    model = ET.Element("model", {"name": name})
    add_text_element(model, "static", "true")
    add_text_element(model, "pose", fmt_pose(x, y, yaw))
    link = ET.SubElement(model, "link", {"name": "link"})
    add_box_visual(
        link,
        name="water_patch",
        pose="0 0 0.005 0 0 0",
        size=WATER_PATCH_SIZE,
        ambient="0.10 0.35 0.85 0.82",
        diffuse="0.10 0.35 0.85 0.82",
        specular="0.95 0.95 0.95 1",
    )
    return model


def main() -> int:
    args = parse_args()
    root = ET.parse(args.world).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"Invalid SDF world file: {args.world}")

    wall_boxes = load_wall_boxes(world) if args.infer_door_yaw else []

    if args.clear_existing:
        existing = [model for model in world.findall("model") if model.get("name", "").startswith("semantic_marker_")]
        for model in existing:
            world.remove(model)

    door_points = load_points(args.door_json)
    dead_end_points = load_points(args.dead_end_json)
    water_points = load_points(args.water_json)

    for idx, point in enumerate(door_points):
        yaw = point["yaw"]
        if args.infer_door_yaw and abs(yaw) < 1e-6:
            yaw = infer_door_yaw(point["x"], point["y"], wall_boxes)
        world.append(build_door_marker(f"semantic_marker_door_{idx}", point["x"], point["y"], yaw))
    for idx, point in enumerate(dead_end_points):
        world.append(
            build_dead_end_marker(
                f"semantic_marker_dead_end_{idx}",
                point["x"],
                point["y"],
                point["yaw"],
            )
        )
    for idx, point in enumerate(water_points):
        world.append(build_water_marker(f"semantic_marker_water_{idx}", point["x"], point["y"], point["yaw"]))

    indent_xml(root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=False)

    print(f"Wrote {args.output}")
    print(
        "markers="
        f"door:{len(door_points)} "
        f"dead_end:{len(dead_end_points)} "
        f"water:{len(water_points)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
