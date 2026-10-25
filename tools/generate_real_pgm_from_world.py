#!/usr/bin/env python3
"""Generate a static occupancy PGM for the real world from SDF wall boxes.

The script keeps the free/unknown mask from an existing reference map and
redraws occupied wall cells from ``real_semantic.world``. It is intended for
updating the ground-truth static map after editing Gazebo wall geometry.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
REAL_DIR = ROOT / "src" / "simulation_bringup" / "world" / "real"


def parse_pose(text: str | None) -> tuple[float, float, float]:
    vals = [float(v) for v in (text or "").split()]
    vals += [0.0] * (6 - len(vals))
    return vals[0], vals[1], vals[5]


def compose_pose(
    parent: tuple[float, float, float],
    child: tuple[float, float, float],
) -> tuple[float, float, float]:
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c = math.cos(pyaw)
    s = math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pyaw + cyaw


def read_yaml_meta(path: Path) -> dict[str, object]:
    meta: dict[str, object] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if key == "resolution":
            meta[key] = float(value)
        elif key == "origin":
            nums = [float(v) for v in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", value)]
            meta[key] = nums[:3]
        else:
            meta[key] = value
    return meta


def world_to_pixel(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    height: int,
) -> tuple[float, float]:
    col = (x - origin_x) / resolution
    row = height - (y - origin_y) / resolution
    return col, row


def box_polygon(
    center: tuple[float, float, float],
    size_x: float,
    size_y: float,
    inflate: float,
) -> list[tuple[float, float]]:
    x, y, yaw = center
    hx = 0.5 * size_x + inflate
    hy = 0.5 * size_y + inflate
    c = math.cos(yaw)
    s = math.sin(yaw)
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in corners]


def iter_wall_boxes(world_path: Path) -> list[tuple[tuple[float, float, float], float, float]]:
    root = ET.parse(world_path).getroot()
    boxes: list[tuple[tuple[float, float, float], float, float]] = []
    for model in root.findall(".//model"):
        model_name = model.attrib.get("name", "")
        if model_name.startswith("semantic_marker"):
            continue
        model_pose = parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            link_name = link.attrib.get("name", "")
            if not link_name.startswith("Wall_"):
                continue
            link_pose = compose_pose(model_pose, parse_pose(link.findtext("pose")))
            collision = link.find("collision")
            if collision is None:
                continue
            size_text = collision.findtext("geometry/box/size")
            if not size_text:
                continue
            size_vals = [float(v) for v in size_text.split()]
            if len(size_vals) < 2:
                continue
            box_pose = compose_pose(link_pose, parse_pose(collision.findtext("pose")))
            boxes.append((box_pose, size_vals[0], size_vals[1]))
    return boxes


def write_yaml(path: Path, image_name: str, meta: dict[str, object]) -> None:
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    resolution = meta.get("resolution", 0.05)
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {resolution}",
                f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, default=REAL_DIR / "real_semantic.world")
    parser.add_argument("--yaml", type=Path, default=REAL_DIR / "real.yaml")
    parser.add_argument("--base", type=Path, default=REAL_DIR / "real.pgm")
    parser.add_argument("--out", type=Path, default=REAL_DIR / "real_from_world.pgm")
    parser.add_argument(
        "--inflate",
        type=float,
        default=0.0,
        help="Inflate each wall box by this many meters before rasterization.",
    )
    args = parser.parse_args()

    meta = read_yaml_meta(args.yaml)
    origin_x, origin_y, _ = meta["origin"]
    resolution = float(meta["resolution"])

    img = Image.open(args.base).convert("L")
    pixels = img.load()
    width, height = img.size

    # Preserve unknown cells from the reference map, but clear old occupied cells.
    for y in range(height):
        for x in range(width):
            if pixels[x, y] < 128:
                pixels[x, y] = 254

    draw = ImageDraw.Draw(img)
    boxes = iter_wall_boxes(args.world)
    for pose, size_x, size_y in boxes:
        polygon_world = box_polygon(pose, size_x, size_y, args.inflate)
        polygon_px = [
            world_to_pixel(x, y, origin_x, origin_y, resolution, height)
            for x, y in polygon_world
        ]
        draw.polygon(polygon_px, fill=0)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    img.save(args.out)
    write_yaml(args.out.with_suffix(".yaml"), args.out.name, meta)
    print(f"wall boxes: {len(boxes)}")
    print(f"wrote: {args.out}")
    print(f"wrote: {args.out.with_suffix('.yaml')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
