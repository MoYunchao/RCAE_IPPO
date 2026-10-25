#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def read_pgm_size(path: Path) -> tuple[int, int]:
    with path.open('rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P2', b'P5'):
            raise RuntimeError(f'Unsupported PGM format in {path}: {magic!r}')
        tokens: list[bytes] = []
        while len(tokens) < 2:
            line = f.readline()
            if not line:
                break
            line = line.split(b'#', 1)[0].strip()
            if line:
                tokens.extend(line.split())
        if len(tokens) < 2:
            raise RuntimeError(f'Failed to read PGM size from {path}')
        return int(tokens[0]), int(tokens[1])


def read_map_yaml(path: Path) -> tuple[int, int, float, float, float]:
    text = path.read_text(encoding='utf-8')
    image_match = re.search(r'^image:\s*(.+?)\s*$', text, flags=re.MULTILINE)
    res_match = re.search(r'^resolution:\s*([-+0-9.eE]+)\s*$', text, flags=re.MULTILINE)
    origin_match = re.search(r'^origin:\s*\[([^\]]+)\]\s*$', text, flags=re.MULTILINE)
    if not image_match or not res_match or not origin_match:
        raise RuntimeError(f'Invalid map yaml: {path}')

    image_path = Path(image_match.group(1).strip())
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    if not image_path.exists():
        fallback = path.with_name('world.pgm')
        if fallback.exists():
            image_path = fallback
        else:
            pgm_files = sorted(path.parent.glob('*.pgm'))
            if len(pgm_files) == 1:
                image_path = pgm_files[0]
    width, height = read_pgm_size(image_path)
    origin_values = [float(v.strip()) for v in origin_match.group(1).split(',')]
    if len(origin_values) < 2:
        raise RuntimeError(f'Invalid origin in map yaml: {path}')
    return width, height, float(res_match.group(1)), origin_values[0], origin_values[1]


def parse_pose(pose_text: str):
    try:
        values = [float(v) for v in pose_text.split()]
    except ValueError:
        return None
    if len(values) < 6:
        return None
    return values[:6]


def label_from_model_name(name: str):
    for prefix, label in (
        ('semantic_marker_door_', 'door'),
        ('semantic_marker_dead_end_', 'dead_end'),
        ('semantic_marker_water_', 'water'),
    ):
        if name.startswith(prefix):
            return label
    return None


def extract_region_size(model, label: str, region_sizes: dict[str, tuple[float, float]]):
    size_text = model.findtext('.//visual/geometry/box/size')
    if size_text:
        values = [float(v) for v in size_text.split()]
        if len(values) >= 2:
            return values[0], values[1]
    return region_sizes.get(label)


def load_regions_from_world(
    world_sdf_path: Path,
    region_sizes: dict[str, tuple[float, float]],
):
    root = ET.parse(world_sdf_path).getroot()
    world = root.find('world')
    if world is None:
        raise RuntimeError('Invalid SDF: missing <world> root')

    regions = []
    for model in world.findall('model'):
        name = model.get('name', '')
        if not name.startswith('semantic_marker_'):
            continue

        label = label_from_model_name(name)
        if label is None:
            continue

        pose_values = parse_pose(model.findtext('pose', default='0 0 0 0 0 0'))
        if pose_values is None:
            continue

        size = extract_region_size(model, label, region_sizes)
        if size is None:
            continue

        regions.append({
            'name': name,
            'label': label,
            'x': pose_values[0],
            'y': pose_values[1],
            'yaw': pose_values[5],
            'sx': size[0],
            'sy': size[1],
        })
    return regions


def region_distance_grid(wx: np.ndarray, wy: np.ndarray, region: dict) -> np.ndarray:
    dx = wx[None, :] - np.float32(region['x'])
    dy = wy[:, None] - np.float32(region['y'])
    cos_yaw = np.float32(math.cos(region['yaw']))
    sin_yaw = np.float32(math.sin(region['yaw']))
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    hx = np.float32(region['sx'] * 0.5)
    hy = np.float32(region['sy'] * 0.5)
    dx_out = np.maximum(np.abs(local_x) - hx, np.float32(0.0))
    dy_out = np.maximum(np.abs(local_y) - hy, np.float32(0.0))
    return np.hypot(dx_out, dy_out).astype(np.float32, copy=False)


def influence_grid(distance: np.ndarray, cfg: dict) -> np.ndarray:
    mid_scale = np.float32(max(1e-6, float(cfg['mid_scale_m'])))
    near_scale = np.float32(max(1e-6, float(cfg['near_scale_m'])))
    return (
        np.float32(cfg['mid_amp']) * np.exp(-distance / mid_scale) +
        np.float32(cfg['near_amp']) * np.exp(-distance / near_scale)
    ).astype(np.float32, copy=False)


def build_static_grids(
    *,
    world_sdf_path: Path,
    scoring_config_path: Path,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    semantic_value_scale: float,
    chunk_rows: int,
):
    config = json.loads(scoring_config_path.read_text(encoding='utf-8'))
    label_scores = config['labels']
    label_priority = config['priority']
    semantic_bounds = config['semantic_bounds']
    risk_bounds = config['risk_bounds']
    corridor_semantic_influence = config['corridor_semantic_influence']
    corridor_risk_influence = config['corridor_risk_influence']
    region_sizes = {
        'door': (1.6, 1.4),
        'dead_end': (1.8, 1.8),
    }
    regions = load_regions_from_world(world_sdf_path, region_sizes)
    regions_by_label: dict[str, list[dict]] = {}
    for region in regions:
        regions_by_label.setdefault(region['label'], []).append(region)

    semantic_map = np.empty((height, width), dtype=np.float32)
    risk_map = np.empty((height, width), dtype=np.float32)
    x_coords = (
        origin_x +
        (np.arange(width, dtype=np.float32) + np.float32(0.5)) * np.float32(resolution)
    ).astype(np.float32, copy=False)

    default_semantic_value = (
        float(label_scores['corridor']['semantic_value']) * semantic_value_scale
    )
    default_semantic_risk = float(label_scores['corridor']['risk_level'])
    chunk_rows = max(16, int(chunk_rows))

    for y0 in range(0, height, chunk_rows):
        y1 = min(height, y0 + chunk_rows)
        chunk_h = y1 - y0
        y_coords = (
            origin_y +
            (np.arange(y0, y1, dtype=np.float32) + np.float32(0.5)) *
            np.float32(resolution)
        ).astype(np.float32, copy=False)
        semantic_chunk = np.full(
            (chunk_h, width),
            np.float32(default_semantic_value),
            dtype=np.float32,
        )
        risk_chunk = np.full(
            (chunk_h, width),
            np.float32(default_semantic_risk),
            dtype=np.float32,
        )

        for label, cfg in corridor_semantic_influence.items():
            label_regions = regions_by_label.get(label, [])
            if not label_regions:
                continue
            min_dist = np.full((chunk_h, width), np.inf, dtype=np.float32)
            for region in label_regions:
                np.minimum(
                    min_dist,
                    region_distance_grid(x_coords, y_coords, region),
                    out=min_dist,
                )
            semantic_chunk += np.float32(cfg['sign']) * influence_grid(min_dist, cfg)

        for label, cfg in corridor_risk_influence.items():
            label_regions = regions_by_label.get(label, [])
            if not label_regions:
                continue
            min_dist = np.full((chunk_h, width), np.inf, dtype=np.float32)
            for region in label_regions:
                np.minimum(
                    min_dist,
                    region_distance_grid(x_coords, y_coords, region),
                    out=min_dist,
                )
            risk_chunk += np.float32(cfg['sign']) * influence_grid(min_dist, cfg)

        np.clip(
            semantic_chunk,
            np.float32(semantic_bounds['min']),
            np.float32(semantic_bounds['max']),
            out=semantic_chunk,
        )
        semantic_chunk *= np.float32(semantic_value_scale)
        np.clip(
            risk_chunk,
            np.float32(risk_bounds['min']),
            np.float32(risk_bounds['max']),
            out=risk_chunk,
        )

        priority_chunk = np.full((chunk_h, width), -1, dtype=np.int16)
        for region in regions:
            priority = int(label_priority.get(region['label'], 0))
            if priority < 0:
                continue
            inside = region_distance_grid(x_coords, y_coords, region) <= np.float32(1e-6)
            update_mask = inside & (priority > priority_chunk)
            if not np.any(update_mask):
                continue
            label_score = label_scores[region['label']]
            semantic_chunk[update_mask] = (
                np.float32(label_score['semantic_value']) *
                np.float32(semantic_value_scale)
            )
            risk_chunk[update_mask] = np.float32(label_score['risk_level'])
            priority_chunk[update_mask] = priority

        semantic_map[y0:y1, :] = semantic_chunk
        risk_map[y0:y1, :] = risk_chunk

    counts = {}
    for region in regions:
        counts[region['label']] = counts.get(region['label'], 0) + 1

    return semantic_map.reshape(-1), risk_map.reshape(-1), counts


def default_repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def main() -> int:
    repo = default_repo_root()
    parser = argparse.ArgumentParser(
        description='Build a precomputed static semantic grid for the v2 global_map.'
    )
    parser.add_argument(
        '--world',
        type=Path,
        default=repo / 'src/simulation_bringup/world/model/model_semantic.world',
    )
    parser.add_argument(
        '--scoring',
        type=Path,
        default=repo / 'src/clip_semantic/config/static_semantic_scores.json',
    )
    parser.add_argument(
        '--map-yaml',
        type=Path,
        default=repo / 'src/simulation_bringup/world/model/world.yaml',
        help='Map yaml whose width/height/resolution/origin define the static semantic grid.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=repo / 'src/clip_semantic/config/static_semantic_grid_model.npz',
    )
    parser.add_argument('--width', type=int, default=None)
    parser.add_argument('--height', type=int, default=None)
    parser.add_argument('--resolution', type=float, default=None)
    parser.add_argument('--origin-x', type=float, default=None)
    parser.add_argument('--origin-y', type=float, default=None)
    parser.add_argument('--semantic-value-scale', type=float, default=1.0)
    parser.add_argument('--chunk-rows', type=int, default=128)
    args = parser.parse_args()

    map_width, map_height, map_resolution, map_origin_x, map_origin_y = read_map_yaml(args.map_yaml)
    width = int(args.width if args.width is not None else map_width)
    height = int(args.height if args.height is not None else map_height)
    resolution = float(args.resolution if args.resolution is not None else map_resolution)
    origin_x = float(args.origin_x if args.origin_x is not None else map_origin_x)
    origin_y = float(args.origin_y if args.origin_y is not None else map_origin_y)

    start = time.perf_counter()
    semantic, risk, counts = build_static_grids(
        world_sdf_path=args.world,
        scoring_config_path=args.scoring,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        semantic_value_scale=args.semantic_value_scale,
        chunk_rows=args.chunk_rows,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        semantic=semantic.astype(np.float32, copy=False),
        risk=risk.astype(np.float32, copy=False),
        width=np.int32(width),
        height=np.int32(height),
        resolution=np.float32(resolution),
        origin_x=np.float32(origin_x),
        origin_y=np.float32(origin_y),
        semantic_value_scale=np.float32(args.semantic_value_scale),
        map_yaml_path=str(args.map_yaml),
        world_sdf_path=str(args.world),
        scoring_config_path=str(args.scoring),
    )

    elapsed = time.perf_counter() - start
    print(f'Wrote {args.output}')
    print(
        f'map={width}x{height}, res={resolution}, '
        f'origin=({origin_x}, {origin_y}), regions={counts}'
    )
    print(
        f'semantic min={float(np.min(semantic)):.3f}, '
        f'max={float(np.max(semantic)):.3f}, '
        f'mean={float(np.mean(semantic)):.3f}'
    )
    print(
        f'risk     min={float(np.min(risk)):.3f}, '
        f'max={float(np.max(risk)):.3f}, '
        f'mean={float(np.mean(risk)):.3f}'
    )
    print(f'elapsed={elapsed:.2f}s')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
