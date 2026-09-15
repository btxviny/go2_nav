#!/usr/bin/env python3
"""Offline: bag -> SAM masks -> CLIP embeddings -> 3D projection -> voxel map.

No live ROS needed -- reads the mcap bag with the pure-Python `rosbags` library and
feeds recorded /tf, /tf_static into a standalone tf2_ros.Buffer (see
semantic_map_common.TfBagBuffer), so this only needs the repo-root uv venv activated
(`source .venv/bin/activate`), not `source install/setup.bash`.

For every Nth RGB frame: run FastSAM to get masks + boxes -> for each mask, expand
its box, crop, CLIP-encode the crop -> project every (subsampled, depth-filtered)
pixel in the mask to a 3D point in the `map` frame (pixel+depth -> optical-frame
point -> rotate into camera_face's robot-convention axes -> TF to map at that frame's
own timestamp) -> accumulate into a VoxelGrid with that mask's embedding, weighted by
1/distance-to-voxel-center. See semantic_map_common.py's module docstring for the
optical/camera_face frame-convention detail this projection depends on.

Usage:
    python3 build_semantic_map.py --bag bags/semantic_test --out bags/semantic_test/semantic_map.npz
"""
import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from semantic_map_common import (  # noqa: E402
    CameraModel, TfBagBuffer, VoxelGrid, expand_bbox, optical_to_face, transform_points,
)

IMAGE_TOPIC = '/robot1/camera/image'
DEPTH_TOPIC = '/robot1/camera/depth_image'
CAMERA_INFO_TOPIC = '/robot1/camera/camera_info'
# Namespaced, not bare /tf, /tf_static -- confirmed live via `ros2 topic list`
# against the running stack; see record_bag.sh's matching note.
TF_TOPICS = ('/robot1/tf', '/robot1/tf_static')

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'semantic_map.yaml'


def load_config(path: Path, overrides: dict) -> dict:
    cfg = yaml.safe_load(path.read_text())
    cfg.update({k: v for k, v in overrides.items() if v is not None})
    return cfg


def decode_image(raw_msg) -> np.ndarray:
    """rosbags-decoded sensor_msgs/Image -> numpy array. Only the two encodings this
    sim actually produces (see bridge.yaml) are handled -- rgb8 color, 32FC1 depth."""
    data = np.frombuffer(bytes(raw_msg.data), dtype=np.uint8)
    if raw_msg.encoding == 'rgb8':
        return data.reshape(raw_msg.height, raw_msg.width, 3)
    if raw_msg.encoding == '32FC1':
        return data.view(np.float32).reshape(raw_msg.height, raw_msg.width)
    raise ValueError(f'Unhandled image encoding: {raw_msg.encoding!r}')


def stamp_to_ns(raw_msg) -> int:
    return int(raw_msg.header.stamp.sec) * 1_000_000_000 + int(raw_msg.header.stamp.nanosec)


def build_sam_model(checkpoint: str, device: str):
    from ultralytics import FastSAM
    return FastSAM(checkpoint), device


def run_sam(model, device, rgb: np.ndarray):
    """Automatic ("segment everything") mask generation -- no prompt box/point given.
    retina_masks=True keeps returned masks at the original image resolution so no
    manual resize-back-to-source-image step is needed."""
    results = model(rgb, device=device, retina_masks=True, verbose=False)
    r = results[0]
    if r.masks is None or r.boxes is None:
        return None, None
    masks = r.masks.data.cpu().numpy() > 0.5  # (n, H, W) bool
    boxes = r.boxes.xyxy.cpu().numpy()  # (n, 4)
    return masks, boxes


def build_clip(model_name: str, pretrained: str, device: str):
    import open_clip
    model, _, preprocess = open_clip.create_model_and_transforms(model_name, pretrained=pretrained)
    model = model.to(device).eval()
    return model, preprocess


def clip_encode_crop(model, preprocess, device, crop: np.ndarray) -> np.ndarray:
    import torch
    from PIL import Image
    pil = Image.fromarray(crop)
    tensor = preprocess(pil).unsqueeze(0).to(device)
    with torch.no_grad():
        feat = model.encode_image(tensor)
        feat = feat / feat.norm(dim=-1, keepdim=True)
    return feat.squeeze(0).cpu().numpy().astype(np.float32)


def load_tf_and_camera_info(bag_path: Path, tf_buffer: TfBagBuffer):
    """Pass 1: feed every /tf, /tf_static message into tf_buffer; return the first
    CameraInfo seen (intrinsics are constant for this sim's static-mounted camera)."""
    camera_info_raw = None
    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic in (*TF_TOPICS, CAMERA_INFO_TOPIC)]
        for connection, _timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == TF_TOPICS[0]:
                tf_buffer.add_tf_message(msg, static=False)
            elif connection.topic == TF_TOPICS[1]:
                tf_buffer.add_tf_message(msg, static=True)
            elif camera_info_raw is None:
                camera_info_raw = msg
    if camera_info_raw is None:
        raise RuntimeError(f'No {CAMERA_INFO_TOPIC} messages found in {bag_path}')
    return CameraModel.from_camera_info(camera_info_raw)


def process_frame(rgb, depth, cam_model, tf_buffer, stamp_sec, stamp_nanosec,
                   sam_model, sam_device, clip_model, clip_preprocess, clip_device,
                   voxel_grid, cfg):
    try:
        tf = tf_buffer.lookup('map', 'camera_face', stamp_sec, stamp_nanosec)
    except Exception as exc:  # noqa: BLE001 -- tf2 raises several distinct exception types
        print(f'  skip frame @ {stamp_sec}.{stamp_nanosec:09d}: TF lookup failed: {exc}')
        return 0, 0

    masks, boxes = run_sam(sam_model, sam_device, rgb)
    if masks is None:
        return 0, 0

    stride = cfg['mask_point_stride']
    n_points_added = 0
    n_masks_used = 0
    for mask, box in zip(masks, boxes):
        x1, y1, x2, y2 = expand_bbox(box, cfg['bbox_pad_px'], rgb.shape)
        if x2 <= x1 or y2 <= y1:
            continue
        crop = rgb[y1:y2, x1:x2]
        embedding = clip_encode_crop(clip_model, clip_preprocess, clip_device, crop)

        ys, xs = np.nonzero(mask)
        if len(ys) == 0:
            continue
        ys, xs = ys[::stride], xs[::stride]
        depths = depth[ys, xs]
        valid = np.isfinite(depths) & (depths >= cfg['min_depth_m']) & (depths <= cfg['max_depth_m'])
        ys, xs, depths = ys[valid], xs[valid], depths[valid]
        if len(ys) == 0:
            continue

        p_opt = cam_model.pixel_to_optical_xyz(xs, ys, depths)
        p_face = optical_to_face(p_opt)
        p_map = transform_points(tf, p_face)

        for p in p_map:
            voxel_grid.add_point(p, embedding)
        n_points_added += len(p_map)
        n_masks_used += 1

    return n_points_added, n_masks_used


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Path to the recorded rosbag directory')
    parser.add_argument('--out', required=True, help='Output .npz path for the voxel map')
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--every-n', type=int, default=None, help='Override every_nth_frame')
    parser.add_argument('--voxel-size', type=float, default=None, help='Override voxel_size_m')
    parser.add_argument('--device', default=None, help='cuda or cpu (default: auto-detect)')
    args = parser.parse_args()

    cfg = load_config(Path(args.config), {'every_nth_frame': args.every_n, 'voxel_size_m': args.voxel_size})

    import torch
    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'Using device: {device}')

    bag_path = Path(args.bag)
    tf_buffer = TfBagBuffer()
    print('Pass 1/2: loading TF + camera intrinsics...')
    cam_model = load_tf_and_camera_info(bag_path, tf_buffer)

    print('Loading FastSAM...')
    sam_model, sam_device = build_sam_model(cfg['sam_checkpoint'], device)
    print('Loading CLIP...')
    clip_model, clip_preprocess = build_clip(cfg['clip_model'], cfg['clip_pretrained'], device)

    voxel_grid = VoxelGrid(voxel_size_m=cfg['voxel_size_m'])

    print('Pass 2/2: processing frames...')
    n_frames_total = 0
    n_frames_processed = 0
    n_masks_total = 0
    n_points_total = 0
    latest_depth = None
    latest_depth_stamp_ns = None
    t_start = time.time()

    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic in (IMAGE_TOPIC, DEPTH_TOPIC)]
        for connection, _timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == DEPTH_TOPIC:
                latest_depth = decode_image(msg)
                latest_depth_stamp_ns = stamp_to_ns(msg)
                continue

            # IMAGE_TOPIC
            n_frames_total += 1
            if (n_frames_total - 1) % cfg['every_nth_frame'] != 0:
                continue
            if latest_depth is None:
                continue
            stamp_ns = stamp_to_ns(msg)
            if abs(stamp_ns - latest_depth_stamp_ns) > 200_000_000:  # 200ms
                print(f'  skip frame @ {stamp_ns}: nearest depth is '
                      f'{abs(stamp_ns - latest_depth_stamp_ns) / 1e6:.0f}ms away')
                continue

            rgb = decode_image(msg)
            n_points, n_masks = process_frame(
                rgb, latest_depth, cam_model, tf_buffer,
                msg.header.stamp.sec, msg.header.stamp.nanosec,
                sam_model, sam_device, clip_model, clip_preprocess, device,
                voxel_grid, cfg)
            n_frames_processed += 1
            n_points_total += n_points
            n_masks_total += n_masks

            if n_frames_processed % 10 == 0:
                elapsed = time.time() - t_start
                print(f'  {n_frames_processed} frames processed '
                      f'({n_frames_total} seen, {elapsed:.0f}s elapsed, '
                      f'{len(voxel_grid)} voxels so far)')

    print(f'Finalizing voxel grid ({len(voxel_grid)} voxels)...')
    voxel_grid.finalize()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    voxel_grid.save(str(out_path))

    meta = {
        'voxel_size_m': cfg['voxel_size_m'],
        'clip_model': cfg['clip_model'],
        'clip_pretrained': cfg['clip_pretrained'],
        'embedding_dim': int(voxel_grid.embeddings.shape[1]) if voxel_grid.embeddings.size else 0,
        'source_bag': str(bag_path),
        'built_at': datetime.now(timezone.utc).isoformat(),
        'n_frames_processed': n_frames_processed,
        'n_voxels': int(voxel_grid.voxel_centers.shape[0]),
    }
    out_path.with_suffix('.meta.json').write_text(json.dumps(meta, indent=2))

    print(f'Done: {n_frames_processed} frames processed (of {n_frames_total} seen), '
          f'{n_masks_total} masks used, {n_points_total} points projected, '
          f'{voxel_grid.voxel_centers.shape[0]} voxels.')
    print(f'Saved {out_path} and {out_path.with_suffix(".meta.json")}')


if __name__ == '__main__':
    main()
