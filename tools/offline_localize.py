#!/usr/bin/env python3
"""Read-only, seeded, stationary MID-360S scan-to-PLY trial (no ROS graph)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

from offline_lio_replay import _bag_info


def load_map(path):
    with path.open('rb') as stream:
        header = []
        for _ in range(16):
            line = stream.readline(192).decode('ascii').strip()
            if line == 'end_header':
                break
            if not line.startswith('comment '):
                header.append(line)
        else:
            raise ValueError('Invalid PLY header')
        if (len(header) != 6 or header[:2] != ['ply', 'format binary_little_endian 1.0'] or
                not header[2].startswith('element vertex ') or
                header[3:] != ['property float x', 'property float y', 'property float z']):
            raise ValueError('Expected a project binary XYZ PLY in metres')
        count = int(header[2].split()[-1])
        if not 100 <= count <= 2_000_000 or path.stat().st_size - stream.tell() != count * 12:
            raise ValueError('Invalid or unsupported map size')
        points = np.fromfile(stream, '<f4').reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite map')
    return points


def read_scans(bag, start):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from livox_ros_driver2.msg import CustomMsg

    duration, _ = _bag_info(bag)
    if not math.isfinite(start) or not 0 <= start < duration - 1:
        raise ValueError('Select a stationary window within the bag (at least 1.2 s)')
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    begin = info['starting_time']['nanoseconds_since_epoch'] + round(start * 1e9)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic = '/livox/lidar'
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    reader.seek(begin)
    timestamps, scans = [], []
    while reader.has_next() and len(scans) < 3:
        name, raw, t = reader.read_next()
        due = begin + len(scans) * 500_000_000
        if name != topic or t < due:
            continue
        if t - due > 250_000_000:
            raise ValueError('Lidar gap in selected window; use a new stationary window')
        msg = deserialize_message(raw, CustomMsg)
        if (msg.header.frame_id != 'livox_frame' or msg.point_num != len(msg.points)
                or not msg.points):
            raise ValueError('Unexpected LiDAR frame or invalid CustomMsg')
        src = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if timestamps and not 250_000_000 <= src - timestamps[-1] <= 750_000_000:
            raise ValueError('Inconsistent LiDAR source timestamps')
        timestamps.append(src)
        points = np.asarray([(p.x, p.y, p.z) for p in msg.points if p.line < 4 and
                             (p.tag & 0x30) in (0, 0x10)], dtype=np.float32).reshape(-1, 3)
        ranges = np.linalg.norm(points, axis=1)
        scans.append(points[(ranges >= 1) & (ranges <= 20)])
    if len(scans) != 3:
        raise ValueError('Three stationary scans spanning one second are required')
    return timestamps, scans


def rotation(degrees):
    roll, pitch, yaw = np.radians(degrees)
    axes = np.eye(3)
    rx, ry, rz = (cv2.Rodrigues(v * a)[0] for v, a in zip(axes, (roll, pitch, yaw)))
    return rz @ ry @ rx


def angle_deg(r):
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1))))


class Matcher:
    def __init__(self, points):
        self.map = np.ascontiguousarray(points, dtype=np.float32)
        self.index = cv2.flann_Index(self.map, dict(algorithm=1, trees=4))

    def nearest(self, xyz, k):
        return self.index.knnSearch(np.ascontiguousarray(xyz, dtype=np.float32), k,
                                    params=dict(checks=48))

    def fit(self, scan, seed):
        # One point per occupied voxel limits overweighting the floor or nearby walls.
        _, idx = np.unique(np.floor(scan / .12).astype(np.int32), axis=0, return_index=True)
        ids = np.random.default_rng(81).permutation(idx)[:5500]
        if len(ids) < 400:
            raise ValueError('Too few spatially distinct scan points')
        validation, sample = scan[ids[::4]], scan[np.delete(ids, np.s_[::4])]
        r0, t0 = rotation(seed[3:]), np.array(seed[:3], dtype=float)
        r, t = r0.copy(), t0.copy()
        for gate in (.75, .40, .22):
            for _ in range(12):
                q = sample @ r.T
                world = q + t
                neighbors, d2 = self.nearest(world, 8)
                take = (d2[:, 0] < gate**2) & (d2[:, -1] < (gate + .55)**2)
                if take.sum() < 80:
                    return None, {'reason': 'insufficient_nearby_map_points'}
                clusters = self.map[neighbors[take]].astype(float)
                centers = clusters.mean(axis=1)
                centered = clusters - centers[:, None, :]
                eig, vectors = np.linalg.eigh(centered.transpose(0, 2, 1) @ centered)
                planar = (eig[:, 1] > 1e-5) & (eig[:, 0] < .18 * eig[:, 1])
                n = vectors[planar, :, 0]
                err = np.sum(n * (world[take][planar] - centers[planar]), axis=1)
                j = np.column_stack((np.cross(q[take][planar], n), n))
                if len(j) < 80:
                    return None, {'reason': 'too_few_surface_matches'}
                w = np.sqrt(np.minimum(1, .08 / np.maximum(np.abs(err), 1e-9)))
                # Scale rotations to metres at a representative range for conditioning.
                a = j * w[:, None]
                a[:, :3] /= 4
                dx, _, _, sv = np.linalg.lstsq(a, -w * err, rcond=None)
                if sv[-1] < .03 * sv[0]:
                    return None, {'reason': 'insufficient_3d_geometry'}
                dr, dt = dx[:3] / 4, dx[3:]
                r = cv2.Rodrigues(dr)[0] @ r
                t = t + dt
                if np.linalg.norm(dr) < .0003 and np.linalg.norm(dt) < .002:
                    break
        _, d2 = self.nearest(validation @ r.T + t, 1)
        distances = np.sqrt(d2.ravel())
        support, med = float(np.mean(distances <= .20)), float(np.median(distances))
        move, turn = float(np.linalg.norm(t - t0)), angle_deg(r @ r0.T)
        metrics = dict(held_out_points=len(validation), fraction_within_20cm=support,
                       median_distance_m=med, seed_shift_m=move, seed_turn_deg=turn)
        if support < .65 or med > .12 or move > 1.0 or turn > 20:
            return None, dict(metrics, reason='overlap_residual_or_seed_gate')
        return (r, t), metrics


def trial(matcher, scans, seed):
    fits = [matcher.fit(scan, seed) for scan in scans]
    output = {'status': 'rejected', 'scan_checks': [m for _, m in fits]}
    if any(p is None for p, _ in fits):
        return dict(output, reason='one_or_more_scans_rejected')
    poses = [p for p, _ in fits]
    pairs = [(a, b) for i, a in enumerate(poses) for b in poses[:i]]
    shift = max(float(np.linalg.norm(a[1] - b[1])) for a, b in pairs)
    turn = max(angle_deg(a[0] @ b[0].T) for a, b in pairs)
    output.update(max_pair_shift_m=shift, max_pair_angle_deg=turn)
    if shift > .12 or turn > 2:
        return dict(output, reason='stationary_poses_disagree')
    r, t = poses[1]  # Actual middle observation, not an average of orientations.
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = r, t
    return dict(output, status='candidate', xyz_m=t.tolist(), T_map_lidar=matrix.tolist())


def write_overlay(path, target, scan, pose):
    """Orthographic projections with shared metre scale, centred on the candidate."""
    from PIL import Image, ImageDraw

    m = np.asarray(pose)
    t = m[:3, 3]
    view = np.asarray(scan) @ m[:3, :3].T
    local = target - t
    local = local[np.linalg.norm(local, axis=1) <= 20]
    width, radius = 480, 10.0
    image = Image.new('RGB', (3 * width, width + 54), 'white')
    draw = ImageDraw.Draw(image)
    for i, (name, axes) in enumerate((('XY', [0, 1]), ('XZ', [0, 2]), ('YZ', [1, 2]))):
        for cloud, color in ((local, '#829eaf'), (view, '#cd5000')):
            xy = cloud[:, axes]
            xy = xy[np.all(np.abs(xy) < radius, axis=1)]
            xy = np.rint(xy * [23, -23] + [width * (i + .5), width / 2 + 20]).astype(int)
            draw.point([tuple(v) for v in xy.tolist()], fill=color)
        draw.text((width * i + 12, 5), f'{name}   20 x 20 m', fill='black')
        draw.line((width * i, 20, width * i, width + 20), fill='#d1d9df')
    draw.text((12, width + 28), 'Blue-grey: reference map | Orange: new scan | Centre: estimated LiDAR origin', fill='black')
    with path.open('xb') as out:
        image.save(out, format='PNG')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', type=Path, required=True)
    p.add_argument('--bag', type=Path, required=True, help='stationary ROS 2 SQLite bag directory')
    p.add_argument('--start-s', type=float, default=2., help='offset from bag start, seconds')
    p.add_argument('--seed', type=float, required=True, nargs=6,
                   metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'))
    p.add_argument('--report', type=Path, required=True, help='new JSON file, existing parent required')
    p.add_argument('--overlay', type=Path, help='optional new PNG file, only written for a candidate')
    args = p.parse_args()
    if args.overlay and (args.overlay.exists() or args.overlay.resolve() == args.report.resolve()):
        p.error('overlay and report must be distinct new files')
    if not math.isfinite(args.start_s) or args.start_s < 0:
        p.error('start-s must be finite and nonnegative')
    if not np.isfinite(args.seed).all():
        p.error('seed must be finite (metres and degrees)')
    with args.report.open('x') as out:
        result = dict(status='error', map=str(args.map), bag=str(args.bag), seed_m_deg=args.seed,
                      start_s=args.start_s)
        try:
            cv2.setRNGSeed(81)
            matcher = Matcher(load_map(args.map))
            result['map_sha256'] = hashlib.sha256(args.map.read_bytes()).hexdigest()
            stamps, scans = read_scans(args.bag, args.start_s)
            result['lidar_stamps_ns'] = stamps
            result.update(trial(matcher, scans, args.seed))
            if result['status'] == 'candidate' and args.overlay:
                write_overlay(args.overlay, matcher.map, scans[1], result['T_map_lidar'])
                result['overlay'] = str(args.overlay)
        except (OSError, ImportError, ValueError, RuntimeError, KeyError, cv2.error) as error:
            result.update(status='error', reason=str(error))
        json.dump(result, out, ensure_ascii=False, allow_nan=False, indent=2)
        out.write('\n')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
    return {'candidate': 0, 'rejected': 2, 'error': 1}[result['status']]


if __name__ == '__main__':
    try:
        sys.exit(main())
    except OSError as exc:
        sys.exit(f'Cannot write new localization report: {exc}')
