#!/usr/bin/env python3
"""Interactive semantic-map visualizer: text query -> CLIP embedding -> top-K
matching voxels, shown live in a Rerun viewer colored by normalized similarity
(viridis colormap -- best match is yellow, the weakest of the top-K is purple;
everything in between follows viridis' purple->blue->green->yellow ramp).

Runs entirely inside the uv venv -- no ROS needed. Loads a .npz/.meta.json pair
built by build_semantic_map.py and a CLIP text tower; nothing else.

Usage:
    python3 visualize_semantic_map.py --map /path/to/semantic_map.npz
    # then type queries at the prompt, one per line -- each one replaces the
    # highlighted points in the already-open Rerun viewer. Empty line or Ctrl+D
    # to quit.

    python3 visualize_semantic_map.py --map ... --query "orange cone"  # one-shot
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rerun as rr
from matplotlib import colormaps

sys.path.insert(0, str(Path(__file__).resolve().parent))
from semantic_map_common import VoxelGrid, cosine_topk, encode_query  # noqa: E402

# Dim, translucent gray -- spatial context for where the top-K sit within the whole
# map, deliberately unobtrusive so the viridis-colored top-K points read clearly.
BASE_CLOUD_COLOR = (90, 90, 90, 60)
BASE_POINT_RADIUS = 0.02
TOPK_POINT_RADIUS = 0.06


def viridis_colors(normalized: np.ndarray) -> np.ndarray:
    """normalized: (N,) floats in [0,1], 1.0 = best. Returns (N,3) uint8 RGB."""
    cmap = colormaps['viridis']
    rgba = cmap(normalized)  # (N,4) floats in [0,1]
    return (rgba[:, :3] * 255).astype(np.uint8)


def show_query(voxel_map: VoxelGrid, query_embedding: np.ndarray, k: int, query_text: str):
    idx, sims = cosine_topk(voxel_map.embeddings, query_embedding, k)
    if len(idx) == 0:
        print('No voxels in the map.')
        return

    # Min-max normalize *within this query's own top-K*, not against the whole map's
    # similarity range -- so the best-of-K is always yellow and the worst-of-K is
    # always purple, regardless of how strong or weak the absolute match is.
    lo, hi = float(sims.min()), float(sims.max())
    normalized = (sims - lo) / (hi - lo) if hi > lo else np.ones_like(sims)
    colors = viridis_colors(normalized)

    rr.log('map/topk_matches', rr.Points3D(
        positions=voxel_map.voxel_centers[idx], colors=colors, radii=TOPK_POINT_RADIUS))

    best_i = idx[0]
    c = voxel_map.voxel_centers[best_i]
    print(f'Query: {query_text!r} -- top {len(idx)} shown (sim range [{lo:.3f}, {hi:.3f}])')
    print(f'  best match: sim={sims[0]:.3f} at ({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, dest='map_path')
    parser.add_argument('--k', type=int, default=200, help='How many top matches to highlight')
    parser.add_argument('--query', default=None, help='One-shot query; omit for interactive mode')
    parser.add_argument('--device', default=None)
    args = parser.parse_args()

    meta_path = Path(args.map_path).with_suffix('.meta.json')
    meta = json.loads(meta_path.read_text())
    clip_model, clip_pretrained = meta['clip_model'], meta['clip_pretrained']

    import torch
    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')

    voxel_map = VoxelGrid.load(args.map_path)
    print(f'Loaded {voxel_map.voxel_centers.shape[0]} voxels from {args.map_path} '
          f'(built with {clip_model}/{clip_pretrained})')

    rr.init('semantic_map', spawn=True)
    rr.log('map/all_voxels', rr.Points3D(
        positions=voxel_map.voxel_centers, colors=BASE_CLOUD_COLOR, radii=BASE_POINT_RADIUS),
        static=True)

    def run_query(text: str):
        q = encode_query(clip_model, clip_pretrained, text, device)
        show_query(voxel_map, q, args.k, text)

    if args.query:
        run_query(args.query)
        return

    print("Interactive mode -- type a query and press Enter (empty line or Ctrl+D to quit).")
    while True:
        try:
            text = input('query> ').strip()
        except EOFError:
            print()
            break
        if not text:
            break
        run_query(text)


if __name__ == '__main__':
    main()
