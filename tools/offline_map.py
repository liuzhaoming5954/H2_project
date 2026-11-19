#!/usr/bin/env python3
"""Build and inspect one MID-360S survey on the development computer."""

import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys

import numpy as np

from offline_map_geometry import VoxelMap, endpoint_cloud_overlap, trajectory_metrics
from readonly_capture import stamp_ns


ROOT = Path(__file__).resolve().parents[1]
WINDOW_S = 3.0
WINDOW_NS = int(WINDOW_S * 1_000_000_000)
ROUTE_LABEL = {
    "unknown": "起终点关系尚未确认。请对照现场路线说明解读首末差值。",
    "loop": "操作员确认回到同一标记和朝向。首末差值包含实际站位、朝向复现偏差和里程计漂移。",
    "open": "本段终点与起点不同。首末差值描述路线两端的位姿变化。",
    "stationary": "操作员确认全程静止。首末差值和轨迹长度反映静止时的估计波动。",
}


def locate_bag(source):
    for bag in (source / "capture/bag", source / "bag", source):
        if (bag / "metadata.yaml").is_file():
            return bag
    raise ValueError(f"找不到 bag/metadata.yaml，请提供单段路线、采集会话或 bag 目录：{source}")


def positive_number(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("需要有限正数")
    return result


class Survey:
    """Keep registered points in their world frame and only buffer the tail window."""

    def __init__(self, voxel_size):
        self.map = VoxelMap(voxel_size)
        self.first_cloud = VoxelMap(voxel_size)
        self.tail = deque()
        self.first_stamp = None
        self.stamps = []
        self.positions = []
        self.quaternions = []

    def add_odometry(self, msg):
        pose = msg.pose.pose
        p, q = pose.position, pose.orientation
        self.stamps.append(stamp_ns(msg.header.stamp))
        self.positions.append((p.x, p.y, p.z))
        self.quaternions.append((q.x, q.y, q.z, q.w))

    def add_cloud(self, msg):
        from sensor_msgs_py.point_cloud2 import read_points_numpy

        stamp = stamp_ns(msg.header.stamp)
        xyz = read_points_numpy(msg, field_names=("x", "y", "z"))
        self.map.add(xyz)
        if self.first_stamp is None:
            self.first_stamp = stamp
        if stamp - self.first_stamp <= WINDOW_NS:
            self.first_cloud.add(xyz)
        # Own a compact copy, since read_points_numpy can reference the ROS buffer.
        self.tail.append((stamp, xyz.copy(order="C")))
        while self.tail[0][0] < stamp - WINDOW_NS:
            self.tail.popleft()

    def export(self, output, voxel_size):
        from offline_map_views import write_viewer, write_views

        stamps = np.asarray(self.stamps, dtype=np.int64)
        positions = np.asarray(self.positions, dtype=np.float64)
        rotations = np.asarray(self.quaternions, dtype=np.float64)
        trajectory = trajectory_metrics(stamps, positions, rotations, window_s=WINDOW_S)
        with (output / "trajectory.csv").open("x", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_ns", "elapsed_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"])
            for stamp, pos, rot in zip(stamps, positions, rotations):
                writer.writerow([stamp, (stamp - stamps[0]) / 1e9, *pos, *rot])

        points = self.map.points()
        if not len(points):
            raise ValueError("所有配准点云均为空，无法导出地图。")
        self.map.write_ply(output / "map.ply")
        first = last = None
        overlap = {"available": False}
        if trajectory["windows"]["available"]:
            last_cloud = VoxelMap(voxel_size)
            for _, xyz in self.tail:
                last_cloud.add(xyz)
            first_xyz, last_xyz = self.first_cloud.points(), last_cloud.points()
            # Using at least the map resolution avoids measuring empty sub-voxel detail.
            overlap = endpoint_cloud_overlap(first_xyz, last_xyz, voxel_size_m=max(0.30, voxel_size))
            if overlap["available"]:
                first, last = first_xyz, last_xyz
        images = write_views(output, positions, points, start_points=first, end_points=last)
        return {"trajectory": trajectory, "map": self.map.summary(),
                "endpoint_cloud_overlap": overlap, "images": images,
                "viewer": write_viewer(output, positions, points)}


def site_record(bag):
    capture = bag.parent
    if bag.name == "bag" and capture.name == "capture":
        route = capture.parent
    elif bag.name == "bag" and (capture / "report.json").is_file():
        route = capture
    else:
        route = bag
    result = {"route": str(route), "bag": str(bag)}
    if (route / "route.txt").is_file():
        result["field_notes_file"] = str(route / "route.txt")
    report = capture / "report.json"
    if route != bag and report.is_file():
        result["capture_report"] = str(report)
        try:
            data = json.loads(report.read_text())
            if not isinstance(data, dict):
                raise ValueError("expected JSON object")
        except (OSError, ValueError) as error:
            result["capture_report_warning"] = str(error)
        else:
            result.update(capture_ok=data.get("ok"), capture_failure=data.get("failure"))
            environment = data.get("environment")
            if isinstance(environment, dict):
                result["capture_commit"] = environment.get("git_head")
    return result


def write_report(output, report):
    temp = output / "report.json.tmp"
    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temp.replace(output / "report.json")


def write_summary(output, report):
    traj, voxels = report["trajectory"], report["map"]
    ends, windows = traj["endpoints"], traj["windows"]
    lines = [
        "# 离线建图与闭环检查", "",
        ROUTE_LABEL[report["route_kind"]], "",
        f"FAST-LIO2 输出 {traj['poses']} 个位姿，覆盖 {traj['duration_s']:.2f} 秒。"
        f"点云按 {voxels['voxel_size_m']:.2f} 米体素合并，共保留 {voxels['voxels']:,} 个三维点。", "",
        "| 指标 | 结果 |", "| --- | --- |",
        f"| 首末单帧位置差 / 姿态角差 | {ends['distance_m']:.3f} m / {ends['rotation_deg']:.2f}° |",
        f"| 离起点最远距离 | {traj['max_displacement_from_start_m']:.3f} m |",
        f"| 相邻位姿累计路长（含抖动） | {traj['path_length_m']:.3f} m |",
    ]
    if windows["available"]:
        lines.append(f"| 首尾各 {WINDOW_S:g} 秒代表位姿的位置差 / 姿态角差 | "
                     f"{windows['distance_m']:.3f} m / {windows['rotation_deg']:.2f}° |")
        lines.append(f"| 起始 / 结束窗口内距各自中位位置的最大波动 | "
                     f"{windows['start']['max_distance_from_median_m']:.3f} m / "
                     f"{windows['end']['max_distance_from_median_m']:.3f} m |")
    overlap = report["endpoint_cloud_overlap"]
    if overlap["available"]:
        lines.append(f"| 首 / 末窗口中与对方重合的体素比例 | "
                     f"{overlap['first_coverage']:.1%} / {overlap['last_coverage']:.1%} "
                     f"（栅格 {overlap['voxel_size_m']:.2f} m） |")
    for topic, timing in report["replay"]["source_timing"].items():
        lines.append(f"| {topic} 消息头时间 | 平均 {timing['mean_hz']:.2f} Hz，"
                     f"最大间隔 {timing['max_gap_s']*1000:.1f} ms，"
                     f"超过 {timing['gap_threshold_s']*1000:.1f} ms 的间隔有 "
                     f"{timing['gaps_over_threshold']} 个 |")
    lines.extend(["", f"[打开可旋转的三维点云和轨迹]({report['viewer']})", "",
                  "轨迹和点云均在 FAST-LIO 的 camera_init 起始坐标系中。"
                  "首尾点云直接用里程计对齐。现场朝向、遮挡、视野和栅格大小都会影响重合比例。"
                  "结合路线记录看同一墙面、门框与转角是否重合。", ""])
    if not windows["available"]:
        lines.extend(["首尾窗口的数据不足以独立比较，请查看单帧指标。", ""])
    if report["replay"]["coverage_warning"]:
        lines.extend(["**位姿输出与雷达录制首尾相差超过 1 秒，或中途存在超过 1 秒的位姿空档。"
                      "图中轨迹会连接已有位姿，此次首尾差值仅对应有输出的区间。**", ""])
    if report["source"].get("capture_ok") is False:
        lines.extend(["**现场采集报告标记为失败，请核对采集诊断和路线记录。**", ""])
    if "capture_report_warning" in report["source"]:
        lines.extend(["**现场采集报告无法读取，本次仅按 bag 数据进行分析。**", ""])
    for key, label in (("trajectory", "轨迹"), ("pointcloud", "拼接点云"),
                       ("closure_overlay", "首尾点云叠加")):
        if key in report["images"]:
            lines.extend([f"![{label}]({report['images'][key]})", ""])
    lines.append("[三维点云](map.ply) · [轨迹数据](trajectory.csv) · [详细指标](report.json)")
    (output / "summary.md").write_text("\n".join(lines) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="单段路线、采集会话或 bag 目录")
    parser.add_argument("--route-kind", choices=tuple(ROUTE_LABEL), default="unknown",
                        help="按实际采集情况选择 loop、open、stationary 或 unknown")
    parser.add_argument("--output", type=Path, help="新的分析目录，默认位于项目 data/analysis/")
    parser.add_argument("--voxel-size", type=positive_number, default=0.1, metavar="METERS")
    parser.add_argument("--rate", type=positive_number, default=0.5, help="离线播放倍率，默认 0.5")
    args = parser.parse_args(argv)
    output = None
    report = {"ok": False, "route_kind": args.route_kind, "frame": "camera_init",
              "pose_child_frame": "body", "loop_optimization_applied": False}
    try:
        from offline_lio_replay import run_replay

        bag = locate_bag(args.source.expanduser().resolve()).resolve()
        report["source"] = site_record(bag)
        route = Path(report["source"]["route"])
        default = ROOT / "data/analysis" / (route.name + "-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
        dest = (args.output or default).expanduser().resolve()
        if dest.is_relative_to(route):
            raise ValueError("请把分析目录设在原始采集文件夹之外。")
        dest.mkdir(parents=True, exist_ok=False)
        output = dest
        print(f"分析目录：{output}", flush=True)
        report["status"] = "running"
        write_report(output, report)
        survey = Survey(args.voxel_size)
        report["replay"] = run_replay(bag, output, survey.add_odometry, survey.add_cloud,
                                      domain=167, rate=args.rate)
        report.update(survey.export(output, args.voxel_size))
        write_summary(output, report)
        report.update(ok=True, status="complete")
        write_report(output, report)
        print(f"完成。三维点云和轨迹：{output / 'viewer.html'}\n闭环报告：{output / 'summary.md'}", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report.update(ok=False, status="failed", failure=str(error) or type(error).__name__)
        if output is not None:
            try:
                write_report(output, report)
            except OSError as save_error:
                print(f"无法保存失败报告：{save_error}", file=sys.stderr)
        print("离线建图失败：" + report["failure"], file=sys.stderr)
        return 130 if isinstance(error, KeyboardInterrupt) else 1


if __name__ == "__main__":
    raise SystemExit(main())
