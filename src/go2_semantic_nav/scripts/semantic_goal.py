#!/usr/bin/env python3
"""Online: text query -> closest-matching voxel(s) -> Nav2 NavigateToPose goal.

Loads a voxel map built by build_semantic_map.py, encodes the query with CLIP's text
tower (must match the checkpoint the map was built with -- checked against the
sibling .meta.json), finds the best-matching cluster of voxels by cosine similarity,
and sends a Nav2 goal near it (backed off by standoff_distance_m from the cluster's
centroid, along the line from the centroid to the robot's current position, so Nav2
isn't asked to path into the object itself).

This IS a ros2-launch-only node in the same sense as frontier_explorer.py /
coverage_tour.py -- it does a live map->base_link TF lookup, so run it via
semantic_goal.launch.py, not bare `python3 semantic_goal.py`, unless you're
comfortable with it reading the unnamespaced /tf tree.

Usage:
    ros2 launch go2_semantic_nav semantic_goal.launch.py \\
        query:="orange cone" map_path:=/path/to/semantic_map.npz
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from semantic_map_common import VoxelGrid, cosine_topk, encode_query, send_navigate_to_pose_goal  # noqa: E402

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'semantic_map.yaml'


def load_config(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def cluster_voxels(indices: np.ndarray, candidate_mask: np.ndarray, gap_voxels: int):
    """26-connected flood-fill over the candidate voxels' discrete grid indices.

    The grid is already discretized (voxel_size_m apart), so this is a cheap set-based
    BFS rather than a spatial DBSCAN -- gap_voxels lets two candidates separated by a
    few empty voxels (a small SAM/CLIP miss, not really a different object) still
    join the same cluster. Returns a list of index-arrays (into the original
    `indices`/`candidate_mask`-selected array), one per cluster.
    """
    candidate_idx = np.nonzero(candidate_mask)[0]
    if len(candidate_idx) == 0:
        return []
    coords = indices[candidate_idx]
    coord_to_pos = {tuple(c): i for i, c in enumerate(coords)}
    visited = np.zeros(len(coords), dtype=bool)
    offsets = [
        (dx, dy, dz)
        for dx in range(-gap_voxels, gap_voxels + 1)
        for dy in range(-gap_voxels, gap_voxels + 1)
        for dz in range(-gap_voxels, gap_voxels + 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    clusters = []
    for start in range(len(coords)):
        if visited[start]:
            continue
        stack = [start]
        visited[start] = True
        cluster = []
        while stack:
            i = stack.pop()
            cluster.append(i)
            cx, cy, cz = coords[i]
            for dx, dy, dz in offsets:
                nb = coord_to_pos.get((cx + dx, cy + dy, cz + dz))
                if nb is not None and not visited[nb]:
                    visited[nb] = True
                    stack.append(nb)
        clusters.append(candidate_idx[cluster])
    return clusters


def find_target(voxel_map: VoxelGrid, query_embedding: np.ndarray, cfg: dict):
    """Returns (centroid_xyz, best_similarity) or None if nothing matches confidently."""
    sims = voxel_map.embeddings @ query_embedding
    candidate_mask = sims >= cfg['similarity_threshold']
    if candidate_mask.sum() == 0:
        top_idx, _ = cosine_topk(voxel_map.embeddings, query_embedding, cfg['top_k_fallback'])
        candidate_mask = np.zeros(len(sims), dtype=bool)
        candidate_mask[top_idx] = True

    gap_voxels = max(1, int(round(cfg['cluster_radius_m'] / voxel_map.voxel_size)))
    clusters = cluster_voxels(voxel_map.voxel_indices, candidate_mask, gap_voxels)
    if not clusters:
        return None

    best_cluster, best_score = None, -np.inf
    for cluster in clusters:
        score = float(np.sum(sims[cluster] * voxel_map.weights[cluster]))
        if score > best_score:
            best_cluster, best_score = cluster, score

    weights = sims[best_cluster] * voxel_map.weights[best_cluster]
    weights = np.clip(weights, 1e-6, None)
    centroid = np.average(voxel_map.voxel_centers[best_cluster], axis=0, weights=weights)
    return centroid, float(np.max(sims[best_cluster]))


class SemanticGoalNode(Node):
    def __init__(self):
        super().__init__('semantic_goal')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def robot_xy(self):
        t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        return t.transform.translation.x, t.transform.translation.y


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--query', required=True)
    parser.add_argument('--map', required=True, dest='map_path')
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--device', default=None)
    parser.add_argument('--dry-run', action='store_true',
                         help='Print the computed target instead of sending a Nav2 goal')
    args, _ = parser.parse_known_args()  # ros2 launch appends its own args

    cfg = load_config(Path(args.config))
    map_path = Path(args.map_path)
    meta_path = map_path.with_suffix('.meta.json')
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    clip_model_name = meta.get('clip_model', cfg['clip_model'])
    clip_pretrained = meta.get('clip_pretrained', cfg['clip_pretrained'])
    if meta and (meta.get('clip_model') != cfg['clip_model']
                 or meta.get('clip_pretrained') != cfg['clip_pretrained']):
        print(f'Note: map was built with {clip_model_name}/{clip_pretrained}, '
              f'using that (not this config\'s default) to encode the query.')

    import torch
    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')

    voxel_map = VoxelGrid.load(args.map_path)
    print(f'Loaded {voxel_map.voxel_centers.shape[0]} voxels from {args.map_path}')

    print(f'Encoding query: "{args.query}"')
    query_embedding = encode_query(clip_model_name, clip_pretrained, args.query, device)

    target = find_target(voxel_map, query_embedding, cfg)
    if target is None:
        print('No confident match found for this query -- not sending a goal.')
        return
    centroid, best_sim = target
    print(f'Best match: centroid=({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}) '
          f'similarity={best_sim:.3f}')

    rclpy.init()
    node = SemanticGoalNode()
    robot_xy, last_exc = None, None
    for _ in range(20):  # up to ~10s for the TF listener to receive its first /tf
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            robot_xy = node.robot_xy()
            break
        except Exception as exc:  # noqa: BLE001 -- tf2 raises several distinct exception types
            last_exc = exc
    if robot_xy is None:
        print(f'Could not look up robot pose (is the nav stack running?): {last_exc}')
        rclpy.shutdown()
        return
    robot_x, robot_y = robot_xy

    direction = np.array([robot_x - centroid[0], robot_y - centroid[1]])
    norm = np.linalg.norm(direction)
    direction = direction / norm if norm > 1e-6 else np.array([1.0, 0.0])
    goal_xy = centroid[:2] + direction * cfg['standoff_distance_m']
    goal_yaw = float(np.arctan2(-direction[1], -direction[0]))  # face back toward the centroid

    print(f'Goal: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}), yaw={goal_yaw:.2f} rad')
    if args.dry_run:
        print('(dry run -- not sending)')
        rclpy.shutdown()
        return

    succeeded = send_navigate_to_pose_goal(node, goal_xy[0], goal_xy[1], goal_yaw)
    print('Result: SUCCEEDED' if succeeded else 'Result: FAILED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
