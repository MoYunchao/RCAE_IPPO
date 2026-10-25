#!/usr/bin/env python3
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


WORLD_NAME = "maze40"
WORLD_SIZE_M = 40.0
MAP_RESOLUTION_M = 0.05
MAP_ORIGIN_X = -20.0
MAP_ORIGIN_Y = -20.0
MAP_WIDTH = int(WORLD_SIZE_M / MAP_RESOLUTION_M)
MAP_HEIGHT = int(WORLD_SIZE_M / MAP_RESOLUTION_M)
WALL_THICKNESS_M = 0.15
WALL_HEIGHT_M = 2.5
DOOR_WIDTH_M = 1.6


@dataclass(frozen=True)
class Segment:
    kind: str  # 'h' or 'v'
    pos: float
    start: float
    end: float


@dataclass(frozen=True)
class Marker:
    label: str
    x: float
    y: float
    yaw: float = 0.0


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def segmented_vertical(x: float, y0: float, y1: float, gap_centers: list[float]) -> list[Segment]:
    segments: list[Segment] = []
    current = y0
    half_gap = DOOR_WIDTH_M * 0.5
    for center in sorted(gap_centers):
        gap_start = max(y0, center - half_gap)
        gap_end = min(y1, center + half_gap)
        if gap_start > current:
            segments.append(Segment("v", x, current, gap_start))
        current = max(current, gap_end)
    if current < y1:
        segments.append(Segment("v", x, current, y1))
    return segments


def segmented_horizontal(y: float, x0: float, x1: float, gap_centers: list[float]) -> list[Segment]:
    segments: list[Segment] = []
    current = x0
    half_gap = DOOR_WIDTH_M * 0.5
    for center in sorted(gap_centers):
        gap_start = max(x0, center - half_gap)
        gap_end = min(x1, center + half_gap)
        if gap_start > current:
            segments.append(Segment("h", y, current, gap_start))
        current = max(current, gap_end)
    if current < x1:
        segments.append(Segment("h", y, current, x1))
    return segments


def wall_segments() -> list[Segment]:
    segments: list[Segment] = []

    # Outer boundary.
    segments.extend(
        [
            Segment("h", -19.0, -19.0, 19.0),
            Segment("h", 19.0, -19.0, 19.0),
            Segment("v", -19.0, -19.0, 19.0),
            Segment("v", 19.0, -19.0, 19.0),
        ]
    )

    # Grid-like internal walls with controlled door openings.
    vertical_gaps = {
        -12.0: [-14.5, -6.0, 2.5, 11.0, 15.5],
        -4.0: [-15.0, -2.0, 6.0, 14.5],
        4.0: [-10.5, -0.5, 8.0, 15.0],
        12.0: [-15.0, -6.5, 1.5, 10.5],
    }
    horizontal_gaps = {
        -12.0: [-15.0, -6.0, 3.0, 14.0, 16.5],
        -4.0: [-10.0, -1.5, 7.0, 15.0],
        4.0: [-14.0, -5.0, 2.5, 11.5],
        12.0: [-15.0, -9.0, 0.5, 8.0, 15.0],
    }

    for x, gaps in vertical_gaps.items():
        segments.extend(segmented_vertical(x, -19.0, 19.0, gaps))
    for y, gaps in horizontal_gaps.items():
        segments.extend(segmented_horizontal(y, -19.0, 19.0, gaps))

    # Extra partial walls to create pockets, branches, and clearer dead ends.
    segments.extend(
        [
            Segment("v", -16.0, 12.0, 18.0),
            Segment("h", 16.0, -16.0, -12.0),
            Segment("v", 16.0, -18.0, -12.0),
            Segment("h", -16.0, 12.0, 16.0),
            Segment("v", 16.0, 2.5, 5.5),
            Segment("h", 0.0, -19.0, -15.5),
            Segment("h", 0.0, 12.0, 19.0),
            Segment("v", 8.0, 12.0, 18.0),
            Segment("h", 8.0, 4.0, 8.0),
            Segment("v", -8.0, -10.5, -4.0),
            # Leave a doorway on the eastern side so this branch becomes a
            # reachable pocket instead of a sealed room.
            Segment("h", -8.0, -8.0, -6.2),
        ]
    )

    return segments


def door_markers() -> list[Marker]:
    markers: list[Marker] = []
    vertical_gaps = {
        -12.0: [-14.5, -6.0, 2.5, 11.0, 15.5],
        -4.0: [-15.0, -2.0, 6.0, 14.5],
        4.0: [-10.5, -0.5, 8.0, 15.0],
        12.0: [-15.0, -6.5, 1.5, 10.5],
    }
    horizontal_gaps = {
        -12.0: [-15.0, -6.0, 3.0, 14.0, 16.5],
        -4.0: [-10.0, -1.5, 7.0, 15.0],
        4.0: [-14.0, -5.0, 2.5, 11.5],
        12.0: [-15.0, -9.0, 0.5, 8.0, 15.0],
    }
    for x, gaps in vertical_gaps.items():
        for y in gaps:
            markers.append(Marker("door", x, y, math.pi * 0.5))
    for y, gaps in horizontal_gaps.items():
        for x in gaps:
            markers.append(Marker("door", x, y, 0.0))
    return markers


def dead_end_markers() -> list[Marker]:
    return [
        Marker("dead_end", -17.0, 17.0),
        Marker("dead_end", -17.0, -17.0),
        Marker("dead_end", 17.0, -17.0),
        Marker("dead_end", 17.0, 17.0),
        Marker("dead_end", 17.0, 2.0),
        Marker("dead_end", -17.0, 0.0),
        Marker("dead_end", 9.5, 16.5),
        Marker("dead_end", -9.5, -9.5),
    ]


def water_markers() -> list[Marker]:
    return [
        Marker("water", -14.8, 8.5),
        Marker("water", -7.5, 15.2),
        Marker("water", 1.0, 9.5),
        Marker("water", 15.2, 8.8),
        Marker("water", -14.0, -9.0),
        Marker("water", 8.5, -15.0),
    ]


def add_text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = value
    return elem


def add_ground_plane(world: ET.Element) -> None:
    model = ET.SubElement(world, "model", {"name": "ground_plane"})
    add_text(model, "static", "1")
    link = ET.SubElement(model, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    add_text(plane, "normal", "0 0 1")
    add_text(plane, "size", "100 100")
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    add_text(ode, "mu", "100")
    add_text(ode, "mu2", "50")
    torsional = ET.SubElement(friction, "torsional")
    ET.SubElement(torsional, "ode")
    contact = ET.SubElement(surface, "contact")
    ET.SubElement(contact, "ode")
    ET.SubElement(surface, "bounce")
    add_text(collision, "max_contacts", "10")
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    add_text(visual, "cast_shadows", "0")
    geometry = ET.SubElement(visual, "geometry")
    plane = ET.SubElement(geometry, "plane")
    add_text(plane, "normal", "0 0 1")
    add_text(plane, "size", "100 100")
    material = ET.SubElement(visual, "material")
    script = ET.SubElement(material, "script")
    add_text(script, "uri", "file://media/materials/scripts/gazebo.material")
    add_text(script, "name", "Gazebo/Grey")
    add_text(link, "self_collide", "0")
    add_text(link, "enable_wind", "0")
    add_text(link, "kinematic", "0")


def add_light(world: ET.Element) -> None:
    light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    add_text(light, "cast_shadows", "1")
    add_text(light, "pose", "0 0 10 0 -0 0")
    add_text(light, "diffuse", "0.8 0.8 0.8 1")
    add_text(light, "specular", "0.2 0.2 0.2 1")
    attenuation = ET.SubElement(light, "attenuation")
    add_text(attenuation, "range", "1000")
    add_text(attenuation, "constant", "0.9")
    add_text(attenuation, "linear", "0.01")
    add_text(attenuation, "quadratic", "0.001")
    add_text(light, "direction", "-0.5 0.1 -0.9")
    spot = ET.SubElement(light, "spot")
    add_text(spot, "inner_angle", "0")
    add_text(spot, "outer_angle", "0")
    add_text(spot, "falloff", "0")


def add_world_basics(world: ET.Element) -> None:
    add_text(world, "gravity", "0 0 -9.8")
    add_text(world, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")
    ET.SubElement(world, "atmosphere", {"type": "adiabatic"})
    physics = ET.SubElement(world, "physics", {"type": "ode"})
    add_text(physics, "max_step_size", "0.001")
    add_text(physics, "real_time_factor", "1")
    add_text(physics, "real_time_update_rate", "1000")
    scene = ET.SubElement(world, "scene")
    add_text(scene, "ambient", "0.4 0.4 0.4 1")
    add_text(scene, "background", "0.7 0.7 0.7 1")
    add_text(scene, "shadows", "1")
    audio = ET.SubElement(world, "audio")
    add_text(audio, "device", "default")
    ET.SubElement(world, "wind")
    spherical = ET.SubElement(world, "spherical_coordinates")
    add_text(spherical, "surface_model", "EARTH_WGS84")
    add_text(spherical, "latitude_deg", "0")
    add_text(spherical, "longitude_deg", "0")
    add_text(spherical, "elevation", "0")
    add_text(spherical, "heading_deg", "0")


def add_wall_model(world: ET.Element, segments: list[Segment]) -> None:
    model = ET.SubElement(world, "model", {"name": WORLD_NAME})
    add_text(model, "pose", "0 0 0 0 0 0")

    for idx, seg in enumerate(segments):
        link = ET.SubElement(model, "link", {"name": f"Wall_{idx}"})
        if seg.kind == "h":
            sx = seg.end - seg.start
            sy = WALL_THICKNESS_M
            px = (seg.start + seg.end) * 0.5
            py = seg.pos
        else:
            sx = WALL_THICKNESS_M
            sy = seg.end - seg.start
            px = seg.pos
            py = (seg.start + seg.end) * 0.5

        pose = f"{px:.3f} {py:.3f} 0 0 0 0"
        size = f"{sx:.3f} {sy:.3f} {WALL_HEIGHT_M:.3f}"

        collision = ET.SubElement(link, "collision", {"name": f"Wall_{idx}_Collision"})
        geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(geometry, "box")
        add_text(box, "size", size)
        add_text(collision, "pose", f"0 0 {WALL_HEIGHT_M * 0.5:.3f} 0 0 0")
        add_text(collision, "max_contacts", "10")
        surface = ET.SubElement(collision, "surface")
        contact = ET.SubElement(surface, "contact")
        ET.SubElement(contact, "ode")
        ET.SubElement(surface, "bounce")
        friction = ET.SubElement(surface, "friction")
        torsional = ET.SubElement(friction, "torsional")
        ET.SubElement(torsional, "ode")
        ET.SubElement(friction, "ode")

        visual = ET.SubElement(link, "visual", {"name": f"Wall_{idx}_Visual"})
        add_text(visual, "pose", f"0 0 {WALL_HEIGHT_M * 0.5:.3f} 0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        add_text(box, "size", size)
        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        add_text(script, "uri", "file://media/materials/scripts/gazebo.material")
        add_text(script, "name", "Gazebo/Grey")
        add_text(material, "ambient", "1 1 1 1")
        meta = ET.SubElement(visual, "meta")
        add_text(meta, "layer", "0")

        add_text(link, "pose", pose)
        add_text(link, "self_collide", "0")
        add_text(link, "enable_wind", "0")
        add_text(link, "kinematic", "0")


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
    add_text(visual, "pose", pose)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    add_text(box, "size", f"{size[0]:.2f} {size[1]:.2f} {size[2]:.3f}")
    material = ET.SubElement(visual, "material")
    add_text(material, "ambient", ambient)
    add_text(material, "diffuse", diffuse)
    if specular is not None:
        add_text(material, "specular", specular)


def append_markers(world: ET.Element, markers: list[Marker]) -> None:
    door_size = (1.40, 0.26, 0.01)
    door_line_size = (1.22, 0.03, 0.005)
    dead_size = (0.95, 0.95, 0.01)
    cross_size = (0.83, 0.05, 0.005)
    water_size = (0.80, 0.55, 0.01)

    counts = {"door": 0, "dead_end": 0, "water": 0}
    for marker in markers:
        name = f"semantic_marker_{marker.label}_{counts[marker.label]}"
        counts[marker.label] += 1
        model = ET.SubElement(world, "model", {"name": name})
        add_text(model, "static", "true")
        add_text(model, "pose", f"{marker.x:.3f} {marker.y:.3f} 0 0 0 {marker.yaw:.4f}")
        link = ET.SubElement(model, "link", {"name": "link"})
        if marker.label == "door":
            add_box_visual(
                link,
                name="door_patch",
                pose="0 0 0.005 0 0 0",
                size=door_size,
                ambient="0.96 0.96 0.96 1",
                diffuse="0.96 0.96 0.96 1",
            )
            add_box_visual(
                link,
                name="door_center_line",
                pose="0 0 0.011 0 0 0",
                size=door_line_size,
                ambient="0.15 0.15 0.15 1",
                diffuse="0.15 0.15 0.15 1",
            )
        elif marker.label == "dead_end":
            add_box_visual(
                link,
                name="dead_end_patch",
                pose="0 0 0.005 0 0 0",
                size=dead_size,
                ambient="0.85 0.10 0.10 1",
                diffuse="0.85 0.10 0.10 1",
            )
            add_box_visual(
                link,
                name="cross_bar_a",
                pose="0 0 0.011 0 0 0.78",
                size=cross_size,
                ambient="0.98 0.98 0.98 1",
                diffuse="0.98 0.98 0.98 1",
            )
            add_box_visual(
                link,
                name="cross_bar_b",
                pose="0 0 0.011 0 0 -0.78",
                size=cross_size,
                ambient="0.98 0.98 0.98 1",
                diffuse="0.98 0.98 0.98 1",
            )
        else:
            add_box_visual(
                link,
                name="water_patch",
                pose="0 0 0.005 0 0 0",
                size=water_size,
                ambient="0.10 0.35 0.85 0.82",
                diffuse="0.10 0.35 0.85 0.82",
                specular="0.95 0.95 0.95 1",
            )


def indent(elem: ET.Element, level: int = 0) -> None:
    indent_str = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent_str + "  "
        for child in elem:
            indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = indent_str
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent_str


def world_root(with_markers: bool) -> ET.Element:
    sdf = ET.Element("sdf", {"version": "1.7"})
    world = ET.SubElement(sdf, "world", {"name": "default"})
    add_light(world)
    add_ground_plane(world)
    add_world_basics(world)
    add_wall_model(world, wall_segments())
    if with_markers:
        append_markers(world, door_markers() + dead_end_markers() + water_markers())
    indent(sdf)
    return sdf


def segment_bounds(seg: Segment) -> tuple[float, float, float, float]:
    if seg.kind == "h":
        return seg.start, seg.end, seg.pos - WALL_THICKNESS_M * 0.5, seg.pos + WALL_THICKNESS_M * 0.5
    return seg.pos - WALL_THICKNESS_M * 0.5, seg.pos + WALL_THICKNESS_M * 0.5, seg.start, seg.end


def render_pgm(path: Path) -> None:
    img = np.full((MAP_HEIGHT, MAP_WIDTH), 254, dtype=np.uint8)
    for seg in wall_segments():
        min_x, max_x, min_y, max_y = segment_bounds(seg)
        gx0 = max(0, int(math.floor((min_x - MAP_ORIGIN_X) / MAP_RESOLUTION_M)))
        gx1 = min(MAP_WIDTH, int(math.ceil((max_x - MAP_ORIGIN_X) / MAP_RESOLUTION_M)))
        gy0 = max(0, int(math.floor((min_y - MAP_ORIGIN_Y) / MAP_RESOLUTION_M)))
        gy1 = min(MAP_HEIGHT, int(math.ceil((max_y - MAP_ORIGIN_Y) / MAP_RESOLUTION_M)))
        img[MAP_HEIGHT - gy1 : MAP_HEIGHT - gy0, gx0:gx1] = 0

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(f"P5\n{MAP_WIDTH} {MAP_HEIGHT}\n255\n".encode("ascii"))
        f.write(img.tobytes())


def write_yaml(path: Path) -> None:
    text = (
        f"image: {WORLD_NAME}.pgm\n"
        "mode: trinary\n"
        f"resolution: {MAP_RESOLUTION_M}\n"
        f"origin: [{MAP_ORIGIN_X}, {MAP_ORIGIN_Y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    path.write_text(text, encoding="utf-8")


def write_poses(path: Path) -> None:
    text = """# 40x40 迷宫地图的机器人初始位姿

robot_poses:
  robot_0:
    x: -1.50
    y: 0.00
    z: 0.06
    yaw: 0.0

  robot_1:
    x: 1.50
    y: 2.50
    z: 0.06
    yaw: 0.0

  robot_2:
    x: 1.50
    y: -2.50
    z: 0.06
    yaw: 0.0
"""
    path.write_text(text, encoding="utf-8")


def main() -> int:
    root = repo_root()
    world_dir = root / "src" / "simulation_bringup" / "world" / WORLD_NAME
    config_dir = root / "src" / "simulation_bringup" / "config"

    world_dir.mkdir(parents=True, exist_ok=True)
    config_dir.mkdir(parents=True, exist_ok=True)

    ET.ElementTree(world_root(with_markers=False)).write(
        world_dir / f"{WORLD_NAME}.world",
        encoding="utf-8",
        xml_declaration=False,
    )
    ET.ElementTree(world_root(with_markers=True)).write(
        world_dir / f"{WORLD_NAME}_semantic.world",
        encoding="utf-8",
        xml_declaration=False,
    )
    render_pgm(world_dir / f"{WORLD_NAME}.pgm")
    write_yaml(world_dir / f"{WORLD_NAME}.yaml")
    write_poses(config_dir / f"robot_initial_poses_{WORLD_NAME}.yaml")

    print(f"Wrote {world_dir / (WORLD_NAME + '.world')}")
    print(f"Wrote {world_dir / (WORLD_NAME + '_semantic.world')}")
    print(f"Wrote {world_dir / (WORLD_NAME + '.pgm')}")
    print(f"Wrote {world_dir / (WORLD_NAME + '.yaml')}")
    print(f"Wrote {config_dir / ('robot_initial_poses_' + WORLD_NAME + '.yaml')}")
    print(
        "markers="
        f"door:{len(door_markers())} "
        f"dead_end:{len(dead_end_markers())} "
        f"water:{len(water_markers())}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
