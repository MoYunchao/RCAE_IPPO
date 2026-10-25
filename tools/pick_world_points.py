#!/usr/bin/env python3
import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle
from matplotlib.transforms import Affine2D


@dataclass
class PreviewShape:
    kind: str
    x: float
    y: float
    yaw: float
    sx: float = 0.0
    sy: float = 0.0
    radius: float = 0.0
    source: str = "other"


def normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def world_transform(base_pose, local_pose):
    bx, by, _, _, _, byaw = base_pose
    lx, ly, _, _, _, lyaw = local_pose
    x = bx + math.cos(byaw) * lx - math.sin(byaw) * ly
    y = by + math.sin(byaw) * lx + math.cos(byaw) * ly
    return x, y, normalize_angle(byaw + lyaw)


def count_box_links(model: ET.Element) -> int:
    count = 0
    for link in model.findall("link"):
        collision = link.find("collision")
        if collision is None:
            continue
        if collision.findtext("geometry/box/size"):
            count += 1
    return count


def select_maze_model(world: ET.Element, model_name: str = "") -> ET.Element:
    explicit_name = str(model_name).strip()
    if explicit_name:
        maze_model = world.find(f"model[@name='{explicit_name}']")
        if maze_model is None:
            raise RuntimeError(f"cannot find maze model '{explicit_name}'")
        return maze_model

    legacy_model = world.find("model[@name='Untitled']")
    if legacy_model is not None:
        return legacy_model

    candidates = []
    for model in world.findall("model"):
        score = count_box_links(model)
        if score > 0:
            candidates.append((score, model))

    if not candidates:
        raise RuntimeError("cannot find any model with box collision geometry")

    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]


def iter_collision_shapes(world_path: Path, maze_model_name: str = "", include_all_models: bool = True):
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("invalid sdf: missing <world>")

    maze_model = select_maze_model(world, model_name=maze_model_name)
    maze_name = maze_model.get("name", "")

    for model in world.findall("model"):
        model_name = model.get("name", "")
        if model_name == "ground_plane":
            continue
        if not include_all_models and model_name != maze_name:
            continue

        model_pose = [float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split()]
        source = "maze" if model_name == maze_name else "other"

        for link in model.findall("link"):
            link_pose = [float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()]
            link_world_x, link_world_y, link_world_yaw = world_transform(model_pose, link_pose)

            for collision in link.findall("collision"):
                collision_pose = [float(v) for v in collision.findtext("pose", "0 0 0 0 0 0").split()]
                cx, cy, cyaw = world_transform(
                    [link_world_x, link_world_y, 0.0, 0.0, 0.0, link_world_yaw],
                    collision_pose,
                )

                box_size_text = collision.findtext("geometry/box/size")
                if box_size_text:
                    sx, sy, *_ = (float(v) for v in box_size_text.split())
                    yield PreviewShape(kind="box", x=cx, y=cy, yaw=cyaw, sx=sx, sy=sy, source=source)
                    continue

                cylinder_radius_text = collision.findtext("geometry/cylinder/radius")
                if cylinder_radius_text:
                    radius = float(cylinder_radius_text)
                    yield PreviewShape(kind="cylinder", x=cx, y=cy, yaw=cyaw, radius=radius, source=source)


def compute_bounds(shapes, fallback_bounds, margin: float = 1.0):
    if not shapes:
        return fallback_bounds

    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")

    for shape in shapes:
        if shape.kind == "box":
            c = math.cos(shape.yaw)
            s = math.sin(shape.yaw)
            hx = shape.sx * 0.5
            hy = shape.sy * 0.5
            ex = abs(c) * hx + abs(s) * hy
            ey = abs(s) * hx + abs(c) * hy
        else:
            ex = shape.radius
            ey = shape.radius

        min_x = min(min_x, shape.x - ex)
        max_x = max(max_x, shape.x + ex)
        min_y = min(min_y, shape.y - ey)
        max_y = max(max_y, shape.y + ey)

    return (
        min_x - margin,
        max_x + margin,
        min_y - margin,
        max_y + margin,
    )


def snap(value: float, resolution: float) -> float:
    if resolution <= 0.0:
        return float(value)
    return round(round(value / resolution) * resolution, 3)


def main():
    parser = argparse.ArgumentParser(
        description="Open an SDF world preview, click points, and save them to JSON."
    )
    parser.add_argument("--world", type=Path, required=True, help="Path to .world file")
    parser.add_argument("--output", type=Path, required=True, help="Output JSON path")
    parser.add_argument("--label", type=str, default="marker", help="Label stored in JSON")
    parser.add_argument("--model-name", type=str, default="", help="Optional maze model name")
    parser.add_argument("--resolution", type=float, default=0.05, help="Point snap resolution in meters")
    parser.add_argument("--maze-only-preview", action="store_true", help="Only draw the selected maze model")
    parser.add_argument("--auto-bounds", action="store_true", help="Auto-compute preview bounds from geometry")
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=4,
        default=None,
        metavar=("MIN_X", "MAX_X", "MIN_Y", "MAX_Y"),
        help="Manual preview bounds",
    )
    args = parser.parse_args()

    preview_shapes = list(
        iter_collision_shapes(
            args.world,
            maze_model_name=args.model_name,
            include_all_models=not args.maze_only_preview,
        )
    )

    default_bounds = (-16.5, 16.5, -16.5, 16.5)
    manual_bounds = tuple(args.bounds) if args.bounds is not None else default_bounds
    use_auto_bounds = args.auto_bounds or args.bounds is None
    bounds = compute_bounds(preview_shapes, manual_bounds) if use_auto_bounds else manual_bounds

    points = []

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_facecolor("#f5f5f5")
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Left click: add point | Right click: remove last | Close window: save and exit", fontsize=10)
    ax.set_xlabel(f"world x (snap={args.resolution})")
    ax.set_ylabel("world y")

    for shape in preview_shapes:
        face = "#303030" if shape.source == "maze" else "#8d8d8d"
        edge = "#202020" if shape.source == "maze" else "#666666"
        if shape.kind == "box":
            patch = Rectangle(
                (shape.x - shape.sx * 0.5, shape.y - shape.sy * 0.5),
                shape.sx,
                shape.sy,
                facecolor=face,
                edgecolor=edge,
                linewidth=0.8,
            )
            patch.set_transform(Affine2D().rotate_around(shape.x, shape.y, shape.yaw) + ax.transData)
        else:
            patch = Circle(
                (shape.x, shape.y),
                radius=shape.radius,
                facecolor=face,
                edgecolor=edge,
                linewidth=0.8,
            )
        ax.add_patch(patch)

    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.25)

    scatter = ax.scatter([], [], c="cyan", s=42, edgecolors="black", linewidths=0.6, zorder=10)

    def save_points():
        args.output.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "label": args.label,
            "world": str(args.world),
            "model_name": args.model_name,
            "resolution": float(args.resolution),
            "bounds": {
                "min_x": float(bounds[0]),
                "max_x": float(bounds[1]),
                "min_y": float(bounds[2]),
                "max_y": float(bounds[3]),
            },
            "points": [{"x": p["x"], "y": p["y"]} for p in points],
        }
        args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=True))

    def redraw():
        if not points:
            scatter.set_offsets(np.empty((0, 2)))
        else:
            scatter.set_offsets(np.array([[p["x"], p["y"]] for p in points], dtype=float))
        fig.canvas.draw_idle()

    def on_click(event):
        if event.xdata is None or event.ydata is None:
            return

        x = float(event.xdata)
        y = float(event.ydata)
        if x < bounds[0] or x > bounds[1] or y < bounds[2] or y > bounds[3]:
            return

        if event.button == 3:
            if points:
                removed = points.pop()
                print(f"removed: x={removed['x']:.3f}, y={removed['y']:.3f}")
                save_points()
                redraw()
            return

        point = {
            "x": snap(x, args.resolution),
            "y": snap(y, args.resolution),
        }
        points.append(point)
        print(f"{args.label}: x={point['x']:.3f}, y={point['y']:.3f}")
        save_points()
        redraw()

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.show()

    save_points()
    if points:
        print("final points:")
        for idx, point in enumerate(points):
            print(f"{idx}: x={point['x']:.3f}, y={point['y']:.3f}")
    else:
        print("no points saved")


if __name__ == "__main__":
    main()
