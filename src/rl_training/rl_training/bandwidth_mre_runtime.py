from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class FrontierPoint:
    frontier_id: int
    x: float
    y: float
    gain: float = 0.0
    risk: float = 0.0
    semantic: float = 0.0


@dataclass(frozen=True)
class OnlineGraphObservation:
    node_inputs: object
    node_padding_mask: object
    edge_mask: object
    current_index: object
    current_edge: object
    edge_padding_mask: object
    current_coord: object
    local_xy: np.ndarray
    candidate_local: List[int]
    candidate_xy: List[Tuple[float, float]]
    candidate_utility: List[float]

    @property
    def tensors(self) -> List[object]:
        return [
            self.node_inputs,
            self.node_padding_mask,
            self.edge_mask,
            self.current_index,
            self.current_edge,
            self.edge_padding_mask,
            self.current_coord,
        ]


class RuntimeCommunicationStore:
    def __init__(self, robot_ids: Sequence[int], message_dim: int) -> None:
        self.robot_ids = [int(rid) for rid in robot_ids]
        self.message_dim = int(message_dim)
        self.messages: Dict[int, np.ndarray] = {
            int(rid): np.zeros((self.message_dim,), dtype=np.float32) for rid in self.robot_ids
        }
        self.stamps: Dict[int, float] = {int(rid): 0.0 for rid in self.robot_ids}

    def write(self, robot_id: int, vector: np.ndarray, stamp: float) -> None:
        arr = np.asarray(vector, dtype=np.float32).reshape(-1)
        if arr.shape[0] != self.message_dim:
            raise ValueError(f"message_dim mismatch: expected {self.message_dim}, got {arr.shape[0]}")
        self.messages[int(robot_id)] = arr.copy()
        self.stamps[int(robot_id)] = float(stamp)

    def read_stacked(self, robot_ids: Sequence[int]) -> np.ndarray:
        return np.stack([self.messages.get(int(rid), np.zeros((self.message_dim,), dtype=np.float32)) for rid in robot_ids], axis=0)


class OnlineLocalGraphBuilder:
    def __init__(
        self,
        *,
        node_padding_size: int,
        max_neighbor_actions: int,
        local_radius_m: float,
        decision_min_distance_m: float,
        decision_max_distance_m: float,
        sample_spacing_m: float,
        edge_radius_m: float,
        utility_radius_m: float,
        utility_top_k: int = 5,
        guidepost_top_k: int = 5,
        coord_norm_scale_m: float = 40.0,
        obstacle_threshold: int = 50,
    ) -> None:
        self.node_padding_size = int(node_padding_size)
        self.max_neighbor_actions = int(max_neighbor_actions)
        self.local_radius_m = float(local_radius_m)
        self.decision_min_distance_m = max(0.0, float(decision_min_distance_m))
        self.decision_max_distance_m = max(self.decision_min_distance_m, float(decision_max_distance_m))
        self.sample_spacing_m = float(sample_spacing_m)
        self.edge_radius_m = float(edge_radius_m)
        self.utility_radius_m = float(utility_radius_m)
        self.utility_top_k = max(1, int(utility_top_k))
        self.guidepost_top_k = max(1, int(guidepost_top_k))
        self.coord_norm_scale_m = max(1e-6, float(coord_norm_scale_m))
        self.obstacle_threshold = int(obstacle_threshold)

    @staticmethod
    def _grid_shape(map_msg) -> Tuple[int, int]:
        return int(map_msg.info.width), int(map_msg.info.height)

    def world_to_grid(self, map_msg, x: float, y: float) -> Optional[Tuple[int, int]]:
        res = float(map_msg.info.resolution)
        if res <= 0.0:
            return None
        gx = int(np.floor((float(x) - float(map_msg.info.origin.position.x)) / res))
        gy = int(np.floor((float(y) - float(map_msg.info.origin.position.y)) / res))
        width, height = self._grid_shape(map_msg)
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return None
        return gx, gy

    def grid_to_world(self, map_msg, gx: int, gy: int) -> Tuple[float, float]:
        res = float(map_msg.info.resolution)
        return (
            float(map_msg.info.origin.position.x) + (float(gx) + 0.5) * res,
            float(map_msg.info.origin.position.y) + (float(gy) + 0.5) * res,
        )

    def _value_at(self, map_msg, gx: int, gy: int) -> int:
        width, _ = self._grid_shape(map_msg)
        idx = int(gy) * width + int(gx)
        if idx < 0 or idx >= len(map_msg.data):
            return 100
        return int(map_msg.data[idx])

    def is_known_free_world(self, map_msg, x: float, y: float) -> bool:
        idx = self.world_to_grid(map_msg, x, y)
        if idx is None:
            return False
        value = self._value_at(map_msg, idx[0], idx[1])
        return 0 <= value < self.obstacle_threshold

    def line_is_free(self, map_msg, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        a_idx = self.world_to_grid(map_msg, a[0], a[1])
        b_idx = self.world_to_grid(map_msg, b[0], b[1])
        if a_idx is None or b_idx is None:
            return False
        x0, y0 = a_idx
        x1, y1 = b_idx
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for step in range(steps + 1):
            t = float(step) / float(steps)
            gx = int(round((1.0 - t) * x0 + t * x1))
            gy = int(round((1.0 - t) * y0 + t * y1))
            if not self.is_known_free_world(map_msg, *self.grid_to_world(map_msg, gx, gy)):
                return False
        return True

    def line_has_no_obstacle(self, map_msg, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        a_idx = self.world_to_grid(map_msg, a[0], a[1])
        b_idx = self.world_to_grid(map_msg, b[0], b[1])
        if a_idx is None or b_idx is None:
            return False
        x0, y0 = a_idx
        x1, y1 = b_idx
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for step in range(steps + 1):
            t = float(step) / float(steps)
            gx = int(round((1.0 - t) * x0 + t * x1))
            gy = int(round((1.0 - t) * y0 + t * y1))
            value = self._value_at(map_msg, gx, gy)
            if value >= self.obstacle_threshold:
                return False
        return True

    def _sample_known_free_nodes(self, map_msg, robot_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        spacing = max(float(map_msg.info.resolution), self.sample_spacing_m)
        coords: List[Tuple[float, float]] = [(rx, ry)]
        steps = int(np.ceil(self.local_radius_m / spacing))
        for ix in range(-steps, steps + 1):
            for iy in range(-steps, steps + 1):
                x = rx + float(ix) * spacing
                y = ry + float(iy) * spacing
                if (x - rx) * (x - rx) + (y - ry) * (y - ry) > self.local_radius_m * self.local_radius_m:
                    continue
                if self.is_known_free_world(map_msg, x, y):
                    coords.append((x, y))
        dedup: Dict[Tuple[int, int], Tuple[float, float]] = {}
        for x, y in coords:
            key = (int(round(x / spacing)), int(round(y / spacing)))
            dedup[key] = (float(x), float(y))
        return list(dedup.values())

    def _count_unknown_near(self, map_msg, x: float, y: float, radius_m: float) -> int:
        center = self.world_to_grid(map_msg, x, y)
        if center is None:
            return 0
        res = float(map_msg.info.resolution)
        if res <= 0.0:
            return 0
        radius_cells = max(1, int(np.ceil(float(radius_m) / res)))
        cx, cy = center
        width, height = self._grid_shape(map_msg)
        count = 0
        for gx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
            for gy in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
                if (gx - cx) * (gx - cx) + (gy - cy) * (gy - cy) > radius_cells * radius_cells:
                    continue
                if self._value_at(map_msg, gx, gy) < 0:
                    count += 1
        return int(count)

    def _frontier_gain(self, map_msg, frontiers: Sequence[FrontierPoint]) -> np.ndarray:
        if not frontiers:
            return np.zeros((0,), dtype=np.float32)

        raw_gain = np.asarray([max(0.0, float(f.gain)) for f in frontiers], dtype=np.float32)
        if float(np.max(raw_gain)) > 0.0:
            if float(np.max(raw_gain)) > 1.0:
                raw_gain = raw_gain / float(np.max(raw_gain))
            return np.clip(raw_gain, 0.0, 1.0).astype(np.float32)

        # Self-contained fallback when upstream exploration_gain is unavailable.
        unknown = np.asarray(
            [self._count_unknown_near(map_msg, f.x, f.y, self.utility_radius_m) for f in frontiers],
            dtype=np.float32,
        )
        fxy = np.asarray([(f.x, f.y) for f in frontiers], dtype=np.float32)
        local_frontier_count = np.zeros((len(frontiers),), dtype=np.float32)
        if len(frontiers) > 0:
            for idx in range(len(frontiers)):
                dist = np.linalg.norm(fxy - fxy[idx].reshape(1, 2), axis=1)
                local_frontier_count[idx] = float(np.count_nonzero(dist <= self.utility_radius_m))

        max_unknown = max(1.0, float(np.max(unknown)) if unknown.size else 1.0)
        max_local = max(1.0, float(np.max(local_frontier_count)) if local_frontier_count.size else 1.0)
        gain = 0.75 * (unknown / max_unknown) + 0.25 * (local_frontier_count / max_local)
        return np.clip(gain, 0.0, 1.0).astype(np.float32)

    def _utility(self, map_msg, xy: np.ndarray, frontiers: Sequence[FrontierPoint]) -> np.ndarray:
        utility = np.zeros((xy.shape[0],), dtype=np.float32)
        if not frontiers:
            return utility
        fxy = np.asarray([(f.x, f.y) for f in frontiers], dtype=np.float32)
        gain = self._frontier_gain(map_msg, frontiers)
        radius = max(float(self.utility_radius_m), 1e-6)
        decay = max(radius / 3.0, 1e-6)
        for idx in range(xy.shape[0]):
            dist = np.linalg.norm(fxy - xy[idx].reshape(1, 2), axis=1)
            near = np.flatnonzero(dist <= radius)
            if near.size == 0:
                continue
            contributions = []
            for fid in near:
                if not self.line_has_no_obstacle(map_msg, tuple(xy[idx]), tuple(fxy[int(fid)])):
                    continue
                spatial_weight = float(np.exp(-float(dist[int(fid)]) / decay))
                contributions.append(float(gain[int(fid)]) * spatial_weight)
            if not contributions:
                continue
            contributions.sort(reverse=True)
            utility[idx] = float(sum(contributions[: self.utility_top_k]) / float(self.utility_top_k))
        return np.clip(utility, 0.0, 1.0).astype(np.float32)

    def _guidepost(self, utility: np.ndarray, edge_mask: np.ndarray, node_count: int) -> np.ndarray:
        guidepost = np.zeros((node_count,), dtype=np.float32)
        if node_count <= 1 or utility.size == 0 or float(np.max(utility)) <= 0.0:
            return guidepost

        top_nodes = [
            int(idx)
            for idx in np.argsort(-utility[:node_count])
            if int(idx) != 0 and float(utility[int(idx)]) > 0.0
        ][: self.guidepost_top_k]
        if not top_nodes:
            return guidepost

        prev: Dict[int, Optional[int]] = {0: None}
        queue = [0]
        while queue:
            cur = queue.pop(0)
            neighbors = np.flatnonzero(~edge_mask[cur, :node_count])
            for nxt in neighbors:
                nxt = int(nxt)
                if nxt in prev:
                    continue
                prev[nxt] = cur
                queue.append(nxt)

        for target in top_nodes:
            if target not in prev:
                guidepost[target] = 1.0
                continue
            cur = target
            while cur is not None:
                guidepost[int(cur)] = 1.0
                cur = prev.get(int(cur))
        return guidepost

    def build(self, *, torch_module, map_msg, robot_xy: Tuple[float, float], other_robot_xy: Sequence[Tuple[float, float]], frontiers: Sequence[FrontierPoint], device) -> Optional[OnlineGraphObservation]:
        if map_msg is None or not map_msg.data:
            return None
        nodes = self._sample_known_free_nodes(map_msg, robot_xy)
        if len(nodes) <= 1:
            return None
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        nodes = sorted(nodes, key=lambda p: ((p[0] - rx) ** 2 + (p[1] - ry) ** 2, p[0], p[1]))
        nodes = nodes[: self.node_padding_size]
        if (rx, ry) not in nodes:
            nodes[0] = (rx, ry)
        node_count = len(nodes)
        xy = np.asarray(nodes, dtype=np.float32)
        rel_xy = xy - np.asarray([[rx, ry]], dtype=np.float32)
        rel_xy = rel_xy / self.coord_norm_scale_m
        utility = self._utility(map_msg, xy, frontiers)
        occupancy = np.zeros((node_count,), dtype=np.float32)
        occupancy[0] = -1.0
        for ox, oy in other_robot_xy:
            dist = np.linalg.norm(xy - np.asarray([[ox, oy]], dtype=np.float32), axis=1)
            if dist.size:
                nearest = int(np.argmin(dist))
                if float(dist[nearest]) <= self.sample_spacing_m * 1.5:
                    occupancy[nearest] = 1.0
        node_padding = np.ones((self.node_padding_size,), dtype=np.bool_)
        node_padding[:node_count] = False
        edge_mask = np.ones((self.node_padding_size, self.node_padding_size), dtype=np.bool_)
        for i in range(node_count):
            edge_mask[i, i] = False
            for j in range(i + 1, node_count):
                d = float(np.linalg.norm(xy[i] - xy[j]))
                if d <= self.edge_radius_m and self.line_is_free(map_msg, tuple(xy[i]), tuple(xy[j])):
                    edge_mask[i, j] = False
                    edge_mask[j, i] = False
        guidepost = self._guidepost(utility, edge_mask, node_count)
        features = np.concatenate(
            [rel_xy, utility.reshape(-1, 1), guidepost.reshape(-1, 1), occupancy.reshape(-1, 1)],
            axis=1,
        ).astype(np.float32)

        padded_features = np.zeros((self.node_padding_size, 5), dtype=np.float32)
        padded_features[:node_count] = features

        dist_from_robot = np.linalg.norm(xy - xy[0].reshape(1, 2), axis=1)
        preferred = [
            idx
            for idx in range(1, node_count)
            if self.decision_min_distance_m <= float(dist_from_robot[idx]) <= self.decision_max_distance_m
        ]
        if not preferred:
            preferred = [
                idx
                for idx in range(1, node_count)
                if float(dist_from_robot[idx]) <= self.decision_max_distance_m
            ]
        target_dist = 0.5 * (self.decision_min_distance_m + self.decision_max_distance_m)
        positive = [idx for idx in preferred if float(utility[idx]) > 0.0]
        fallback = [idx for idx in preferred if float(utility[idx]) <= 0.0]
        sort_key = lambda idx: (-float(utility[idx]), abs(float(dist_from_robot[idx]) - target_dist), idx)
        non_current = sorted(positive, key=sort_key) + sorted(fallback, key=sort_key)
        candidate_local = [0] + non_current[: max(0, self.max_neighbor_actions - 1)]
        candidate_xy = [(float(xy[idx, 0]), float(xy[idx, 1])) for idx in candidate_local]
        candidate_utility = [float(utility[idx]) for idx in candidate_local]
        padded_edge = np.zeros((self.max_neighbor_actions,), dtype=np.int64)
        padded_edge[: len(candidate_local)] = np.asarray(candidate_local, dtype=np.int64)
        edge_padding = np.ones((self.max_neighbor_actions,), dtype=np.bool_)
        edge_padding[: len(candidate_local)] = False
        if len(candidate_local) > 1:
            edge_padding[0] = True

        torch = torch_module
        return OnlineGraphObservation(
            node_inputs=torch.as_tensor(padded_features, dtype=torch.float32, device=device).unsqueeze(0),
            node_padding_mask=torch.as_tensor(node_padding, dtype=torch.bool, device=device).reshape(1, 1, self.node_padding_size),
            edge_mask=torch.as_tensor(edge_mask, dtype=torch.bool, device=device).unsqueeze(0),
            current_index=torch.as_tensor([0], dtype=torch.long, device=device).reshape(1, 1, 1),
            current_edge=torch.as_tensor(padded_edge, dtype=torch.long, device=device).reshape(1, self.max_neighbor_actions, 1),
            edge_padding_mask=torch.as_tensor(edge_padding, dtype=torch.bool, device=device).reshape(1, 1, self.max_neighbor_actions),
            current_coord=torch.as_tensor([rx, ry], dtype=torch.float32, device=device).reshape(1, 1, 2),
            local_xy=xy,
            candidate_local=[int(v) for v in candidate_local],
            candidate_xy=candidate_xy,
            candidate_utility=candidate_utility,
        )
