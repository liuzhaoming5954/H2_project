"""Known geometry and file-contract regressions for the offline field tool."""

import base64
import json
from pathlib import Path
import re
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from offline_lio_replay import _observe_source_time, _source_timing
from offline_map import locate_bag, main, site_record
from offline_map_geometry import VoxelMap, endpoint_cloud_overlap, trajectory_metrics
from offline_map_views import write_viewer, write_views


class MapTests(unittest.TestCase):
    def test_imu_source_gap_is_visible_even_with_all_messages_received(self):
        stats = _source_timing(200.)
        for count, stamp in enumerate((1_000_000_000, 1_005_000_000, 1_025_000_000), 1):
            _observe_source_time(stats, stamp, count, "/livox/imu")
        self.assertEqual(stats["gaps_over_threshold"], 1)
        self.assertAlmostEqual(stats["max_gap_s"], .02)
        self.assertAlmostEqual(stats["mean_hz"], 80.)
        with self.assertRaisesRegex(RuntimeError, "Non-increasing"):
            _observe_source_time(stats, stamp, 4, "/livox/imu")

    def test_registered_scans_weighted_and_readable_ply(self):
        cloud = VoxelMap(1.0, max_voxels=2)
        cloud.add(np.array([[.1, .1, .1], [.2, .1, .1], [np.nan, 1, 1]]))
        cloud.add(np.array([[.9, .1, .1], [-.01, .2, .2]]))
        self.assertEqual(cloud.summary()["excluded_nonfinite_points"], 1)
        self.assertEqual(cloud.summary()["voxels"], 2)
        with self.assertRaisesRegex(ValueError, "max_voxels"):
            cloud.add(np.array([[2, 0, 0]]))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.ply"
            cloud.write_ply(path)
            header, payload = path.read_bytes().split(b"end_header\n", 1)
            self.assertIn(b"element vertex 2\n", header)
            xyz = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
            np.testing.assert_allclose(xyz, [[.4, .1, .1], [-.01, .2, .2]], atol=1e-6)

    def test_moved_closed_and_open_trajectories(self):
        stamps = 1_793_000_000_000_000_000 + np.arange(13, dtype=np.int64) * 1_000_000_000
        x = [0, 0, 0, 0, 1, 2, 3, 2, 1, 0, 0, 0, 0]
        p = np.column_stack([x, np.zeros((13, 2))])
        q = np.tile([0., 0., 0., 1.], (13, 1))
        q[1::2] *= -1
        loop = trajectory_metrics(stamps, p, q)
        self.assertEqual(loop["path_length_m"], 6)
        self.assertEqual(loop["windows"]["distance_m"], 0)
        self.assertAlmostEqual(loop["windows"]["rotation_deg"], 0)
        p[9:, 1] = .25
        q[9:] = [0, 0, np.sqrt(.5), np.sqrt(.5)]
        moved = trajectory_metrics(stamps, p, q)
        self.assertAlmostEqual(moved["windows"]["distance_m"], .25)
        self.assertAlmostEqual(moved["windows"]["rotation_deg"], 90)
        self.assertFalse(trajectory_metrics(stamps[:5], p[:5], q[:5])["windows"]["available"])
        with self.assertRaises(ValueError):
            trajectory_metrics(stamps[::-1], p, q)

    def test_endpoint_geometry_without_refitting(self):
        first = np.array([[0., 0., 0.], [0., 1., 0.]])
        self.assertEqual(endpoint_cloud_overlap(first, first)["iou"], 1)
        self.assertEqual(endpoint_cloud_overlap(first, first + [2., 0., 0.])["iou"], 0)
        self.assertFalse(endpoint_cloud_overlap(first, first[:0])["available"])

    def test_standalone_preview_preserves_trajectory_and_endpoints(self):
        track = np.array([[0, 0, 0], [5, 1, 0], [.15, .01, 0]], dtype=np.float32)
        cloud = np.array([[0, 3, 1], [4, 0, 1], [8, 1.1, 2.2]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            images = write_views(out, track, cloud, start_points=cloud, end_points=cloud + .1)
            for image in images.values():
                self.assertTrue((out / image).read_bytes().startswith(b"\x89PNG"))
            page = (out / write_viewer(out, track, cloud)).read_text()
            dataset = re.search(r'<script type="application/json" id="dataset">(.*?)</script>', page)
            self.assertIsNotNone(dataset)
            data = json.loads(dataset.group(1))
            shown = np.frombuffer(base64.b64decode(data["track"]), dtype="<f4").reshape(-1, 3)
            np.testing.assert_array_equal(shown, track)
            self.assertTrue(data["hasOverlay"])

    def test_input_layouts_and_failed_run_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            route = root / "loop-01"
            bag = route / "capture/bag"
            bag.mkdir(parents=True)
            (bag / "metadata.yaml").write_text("not_a_bag: true\n")
            (route / "route.txt").write_text("returned to mark")
            for source in (route, bag.parent, bag):
                self.assertEqual(locate_bag(source), bag)
            self.assertEqual(site_record(bag)["route"], str(route))
            self.assertEqual(main([str(route), "--output", str(route / "new")]), 1)
            self.assertFalse((route / "new").exists())
            output = root / "analysis"
            self.assertEqual(main([str(route), "--output", str(output)]), 1)
            report = json.loads((output / "report.json").read_text())
            self.assertFalse(report["ok"])
            self.assertEqual(report["status"], "failed")
            self.assertIn("Expected a completed bag", report["failure"])
            self.assertFalse((output / "map.ply").exists())
            legacy = root / "old-session"
            (legacy / "bag").mkdir(parents=True)
            (legacy / "bag/metadata.yaml").touch()
            (legacy / "report.json").write_text('{"ok":true}')
            self.assertTrue(site_record(locate_bag(legacy))["capture_ok"])


if __name__ == "__main__":
    unittest.main()
