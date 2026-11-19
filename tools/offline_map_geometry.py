"""Bounded accumulation and uncorrected loop diagnostics in FAST-LIO's world frame."""

from pathlib import Path

import numpy as np


def _resolution(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError("voxel_size_m must be finite and positive")
    return value


def _finite_xyz(xyz):
    array = np.asarray(xyz, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("expected XYZ with shape (N, 3)")
    valid = np.isfinite(array).all(axis=1)
    return array[valid], len(array)


def _voxel_keys(xyz, resolution):
    # Three signed 21-bit voxel coordinates fit in one non-negative int64.
    # At 10 cm this spans over 200 km per axis around the local frame origin.
    cells = np.floor(xyz / resolution)
    bias = 1 << 20
    if np.any(cells < -bias) or np.any(cells >= bias):
        raise ValueError("cloud exceeds local voxel grid extent; check odometry or increase voxel size")
    cells = cells.astype(np.int64) + bias
    return (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]


class VoxelMap:
    """Keep the centroid of all received world points in each occupied voxel."""

    def __init__(self, voxel_size_m=0.1, max_voxels=2_000_000):
        self.voxel_size_m = _resolution(voxel_size_m)
        if not isinstance(max_voxels, (int, np.integer)) or max_voxels < 1:
            raise ValueError("max_voxels must be a positive integer")
        self.max_voxels = int(max_voxels)
        self._indices = {}
        self._sums = np.zeros((0, 3), dtype=np.float64)
        self._counts = np.zeros(0, dtype=np.int64)
        self._minimum = np.full(3, np.inf)
        self._maximum = np.full(3, -np.inf)
        self._frames = self._input_points = self._finite_points = 0

    def _grow(self, needed):
        if needed <= len(self._counts):
            return
        capacity = min(self.max_voxels, max(needed, 1024, 2 * len(self._counts)))
        sums = np.zeros((capacity, 3), dtype=np.float64)
        counts = np.zeros(capacity, dtype=np.int64)
        previous = len(self._indices)
        sums[:previous] = self._sums[:previous]
        counts[:previous] = self._counts[:previous]
        self._sums, self._counts = sums, counts

    def add(self, xyz):
        # world_cloud is already registered. Applying the odometry pose here
        # would transform each scan a second time.
        finite, total = _finite_xyz(xyz)
        if len(finite):
            keys, inverse, counts = np.unique(
                _voxel_keys(finite, self.voxel_size_m), return_inverse=True, return_counts=True
            )
            # Numpy 2.0 can return an extra dimension for return_inverse.
            inverse = inverse.reshape(-1)
            indices = np.fromiter(
                (self._indices.get(k, -1) for k in keys.tolist()), dtype=np.intp, count=len(keys)
            )
            new = indices < 0
            start = len(self._indices)
            needed = start + int(np.count_nonzero(new))
            if needed > self.max_voxels:
                raise ValueError(
                    f"voxel map exceeds max_voxels={self.max_voxels}; "
                    "rerun with larger voxels or a higher memory budget"
                )
            self._grow(needed)
            indices[new] = np.arange(start, needed)
            self._indices.update(zip(keys[new].tolist(), range(start, needed)))
            # indices are unique here, so ordinary advanced assignment sums
            # all frame contributions without a Python object per input point.
            for axis in range(3):
                self._sums[indices, axis] += np.bincount(inverse, weights=finite[:, axis])
            self._counts[indices] += counts
            self._minimum = np.minimum(self._minimum, finite.min(axis=0))
            self._maximum = np.maximum(self._maximum, finite.max(axis=0))
        self._frames += 1
        self._input_points += total
        self._finite_points += len(finite)

    def _centroids(self, start, stop):
        return (self._sums[start:stop] / self._counts[start:stop, None]).astype(np.float32)

    def points(self):
        return self._centroids(0, len(self._indices))

    def write_ply(self, path):
        count = len(self._indices)
        header = (
            "ply\nformat binary_little_endian 1.0\n"
            "comment FAST-LIO registered world points in metres, one centroid per voxel\n"
            f"element vertex {count}\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n"
        )
        with Path(path).open("xb") as stream:
            stream.write(header.encode("ascii"))
            for start in range(0, count, 65_536):
                xyz = self._centroids(start, min(start + 65_536, count))
                stream.write(xyz.astype("<f4", copy=False).tobytes())

    def summary(self):
        return {
            "voxel_size_m": self.voxel_size_m,
            "voxels": len(self._indices), "max_voxels": self.max_voxels,
            "frames": self._frames, "input_points": self._input_points,
            "finite_points": self._finite_points,
            "excluded_nonfinite_points": self._input_points - self._finite_points,
            "bounds_min_m": self._minimum.tolist() if self._finite_points else None,
            "bounds_max_m": self._maximum.tolist() if self._finite_points else None,
        }


def _angles_deg(q, reference):
    # Unit quaternions q and -q represent the same orientation.
    cos_half = np.clip(np.abs(q @ reference), 0.0, 1.0)
    return np.rad2deg(2 * np.arccos(cos_half))


def _endpoint_delta(first_p, last_p, first_q, last_q):
    delta = last_p - first_p
    return {
        "delta_xyz_m": delta.tolist(),
        "distance_m": float(np.linalg.norm(delta)),
        "rotation_deg": float(_angles_deg(last_q, first_q)),
    }


def _window_pose(p, q, stamps):
    # Align quaternion signs before taking a componentwise median. The
    # dominant eigenvector gives a sign-invariant hemisphere reference.
    _, basis = np.linalg.eigh(q.T @ q)
    reference = basis[:, -1]
    aligned = np.where((q @ reference < 0)[:, None], -q, q)
    median_q = np.median(aligned, axis=0)
    norm = np.linalg.norm(median_q)
    median_q = median_q / norm if norm > 1e-12 else reference
    median_p = np.median(p, axis=0)
    return median_p, median_q, {
        "samples": len(p), "first_ns": int(stamps[0]), "last_ns": int(stamps[-1]),
        "position_m": median_p.tolist(), "orientation_xyzw": median_q.tolist(),
        "max_distance_from_median_m": float(np.max(np.linalg.norm(p - median_p, axis=1))),
        "max_rotation_from_median_deg": float(np.max(_angles_deg(q, median_q))),
    }


def trajectory_metrics(stamps_ns, positions_m, quaternions_xyzw, *, window_s=3.0):
    """Compare endpoints of the unmodified estimate, without closing the loop."""
    t = np.asarray(stamps_ns)
    p = np.asarray(positions_m, dtype=np.float64)
    q = np.asarray(quaternions_xyzw, dtype=np.float64)
    if t.ndim != 1 or len(t) < 2 or t.dtype.kind not in "iu" or np.any(t[1:] <= t[:-1]):
        raise ValueError("trajectory needs at least two strictly increasing integer nanosecond stamps")
    if p.shape != (len(t), 3) or q.shape != (len(t), 4):
        raise ValueError("trajectory shapes must be positions (N,3), quaternions XYZW (N,4)")
    if not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError("trajectory contains non-finite values")
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms < 1e-12) or not np.isfinite(norms).all():
        raise ValueError("invalid odometry quaternion")
    q = q / norms[:, None]
    if not np.isfinite(window_s) or window_s <= 0:
        raise ValueError("window_s must be finite and positive")
    window_ns = round(window_s * 1e9)
    first_ns, last_ns = int(t[0]), int(t[-1])
    first = t <= first_ns + window_ns
    last = t >= last_ns - window_ns
    windows = {
        "window_s": float(window_s), "available": False,
        "start": {"samples": int(first.sum())}, "end": {"samples": int(last.sum())},
        "distance_m": None, "rotation_deg": None, "delta_xyz_m": None,
    }
    if last_ns - first_ns > 2 * window_ns and first.sum() >= 3 and last.sum() >= 3:
        first_p, first_q, windows["start"] = _window_pose(p[first], q[first], t[first])
        last_p, last_q, windows["end"] = _window_pose(p[last], q[last], t[last])
        windows.update(_endpoint_delta(first_p, last_p, first_q, last_q))
        windows["available"] = True
    else:
        windows["reason"] = "insufficient duration or fewer than three poses per endpoint window"
    return {
        "poses": len(t), "duration_s": (last_ns - first_ns) * 1e-9,
        "path_length_m": float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))),
        "max_displacement_from_start_m": float(np.max(np.linalg.norm(p - p[0], axis=1))),
        "endpoints": _endpoint_delta(p[0], p[-1], q[0], q[-1]),
        "windows": windows,
        "interpretation": (
            "Uncorrected estimates. Endpoint differences measure closure only when the operator "
            "returned to the same physical sensor pose. Check stationary endpoint windows in "
            "the field notes. Static jitter contributes to the summed path length."
        ),
    }


def endpoint_cloud_overlap(first_xyz, last_xyz, *, voxel_size_m=0.30):
    """Compare occupied cells on one world grid without estimating an alignment."""
    resolution = _resolution(voxel_size_m)
    first, _ = _finite_xyz(first_xyz)
    last, _ = _finite_xyz(last_xyz)
    first_keys = np.unique(_voxel_keys(first, resolution))
    last_keys = np.unique(_voxel_keys(last, resolution))
    common = np.intersect1d(first_keys, last_keys, assume_unique=True).size
    nf, nl = first_keys.size, last_keys.size
    available = bool(nf and nl)
    return {
        "available": available, "voxel_size_m": resolution,
        "first_voxels": nf, "last_voxels": nl, "common_voxels": common,
        "first_coverage": common / nf if available else None,
        "last_coverage": common / nl if available else None,
        "iou": common / (nf + nl - common) if available else None,
        "interpretation": (
            "Descriptive world-grid overlap with no registration correction. Visibility, "
            "head orientation, moving objects, sampling and voxel size affect the result."
        ),
    }
