"""Render equal-scale projections of FAST-LIO output in the camera_init frame."""

import base64
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


PAPER = (249, 250, 247)
INK = (29, 46, 51)
MUTED = (76, 98, 103)
GRID = (222, 229, 228)
TRACK = (43, 63, 105)
START = (24, 116, 142)
END = (204, 89, 40)
BOTH = (90, 53, 117)
CJK = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def _label(chinese, english):
    return chinese if CJK.is_file() else english


def _font(size):
    if CJK.is_file():
        return ImageFont.truetype(str(CJK), size, index=2)
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def _text(draw, xy, content, size=18, fill=INK, anchor="lt"):
    draw.text(xy, content, fill=fill, font=_font(size), anchor=anchor)


def _sample(data, limit, name, *, path=False):
    xyz = np.asarray(data)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz):
        raise ValueError(f"{name} must be a nonempty N x 3 array")
    if len(xyz) > limit:
        # Include both ends even for a long trajectory.
        xyz = xyz[np.linspace(0, len(xyz) - 1, limit, dtype=np.intp)]
    xyz = np.asarray(xyz, dtype=np.float64)
    valid = np.all(np.isfinite(xyz), axis=1)
    if path and not valid.all():
        raise ValueError("trajectory contains a nonfinite position")
    if not valid.any():
        raise ValueError(f"{name} contains no finite preview points")
    return xyz if valid.all() else xyz[valid]


class _Projection:
    def __init__(self, axes, box, low, high):
        self.axes = list(axes)
        self.box = box
        self.size = np.array([box[2] - box[0], box[3] - box[1]])
        center = (low[self.axes] + high[self.axes]) * 0.5
        span = np.maximum((high[self.axes] - low[self.axes]) * 1.12, 1.0)
        self.m_per_px = np.max(span / self.size)
        self.low = center - self.size * self.m_per_px * 0.5
        self.high = center + self.size * self.m_per_px * 0.5

    def pixels(self, xyz):
        projected = xyz[:, self.axes]
        visible = ((projected >= self.low) & (projected <= self.high)).all(axis=1)
        pixels = np.rint((projected[visible] - self.low) / self.m_per_px).astype(np.intp)
        pixels[:, 1] = self.size[1] - pixels[:, 1]
        return pixels, visible


def _ticks(low, high, target):
    rough = (high - low) / target
    power = 10.0 ** np.floor(np.log10(rough))
    step = power * next(n for n in (1, 2, 5, 10) if n >= rough / power)
    return np.arange(np.ceil(low / step), np.floor(high / step) + 1) * step


def _axes(draw, projection):
    x0, y0, x1, y1 = projection.box
    a, b = ["XYZ"[i] for i in projection.axes]
    _text(draw, (x0, 128), f"{a}{b}  " + _label("投影", "projection"), 22)
    _text(draw, (x0 - 2, y0 - 26), f"{b} (m)", 17, MUTED)
    _text(draw, ((x0 + x1) / 2, y1 + 40), f"{a} (m)", 17, MUTED, "mt")
    draw.rectangle(projection.box, fill="white", outline=GRID)
    for value in _ticks(projection.low[0], projection.high[0], 7):
        x = x0 + (value - projection.low[0]) / projection.m_per_px
        draw.line((x, y0, x, y1), fill=GRID)
        _text(draw, (x, y1 + 9), f"{value:g}", 15, MUTED, "mt")
    for value in _ticks(projection.low[1], projection.high[1], 6):
        y = y1 - (value - projection.low[1]) / projection.m_per_px
        draw.line((x0, y, x1, y), fill=GRID)
        _text(draw, (x0 - 8, y), f"{value:g}", 15, MUTED, "rm")


def _figure(title, subtitle, low, high):
    image = Image.new("RGB", (1440, 778), PAPER)
    draw = ImageDraw.Draw(image)
    _text(draw, (42, 23), title, 32)
    _text(draw, (44, 78), subtitle, 19, MUTED)
    draw.line((42, 111, 1398, 111), fill=GRID)
    draw.line((42, 664, 1398, 664), fill=GRID)
    views = [
        _Projection((0, 1), (93, 200, 672, 597), low, high),
        _Projection((0, 2), (807, 200, 1386, 597), low, high),
    ]
    for projection in views:
        _axes(draw, projection)
    return image, views


def _trajectory(image, projection, xyz):
    points, _ = projection.pixels(xyz)
    points += projection.box[:2]
    xy = [tuple(point) for point in points.tolist()]
    draw = ImageDraw.Draw(image)
    if len(xy) > 1:
        draw.line(xy, fill=TRACK, width=3)
    x, y = xy[0]
    draw.ellipse((x - 8, y - 8, x + 8, y + 8), outline="white", width=5)
    draw.ellipse((x - 8, y - 8, x + 8, y + 8), outline=START, width=3)
    x, y = xy[-1]
    draw.line((x - 5, y - 5, x + 5, y + 5), fill=END, width=3)
    draw.line((x + 5, y - 5, x - 5, y + 5), fill=END, width=3)


def _path_key(draw):
    x, y = 55, 695
    draw.ellipse((x - 8, y - 8, x + 8, y + 8), outline=START, width=3)
    _text(draw, (72, y), _label("起点", "Start"), 18, anchor="lm")
    draw.line((177, y - 5, 187, y + 5), fill=END, width=3)
    draw.line((177, y + 5, 187, y - 5), fill=END, width=3)
    _text(draw, (199, y), _label("终点", "End"), 18, anchor="lm")
    draw.line((303, y, 343, y), fill=TRACK, width=3)
    _text(draw, (354, y), _label("雷达组件轨迹", "Sensor trajectory"), 18, anchor="lm")


def _cloud_bounds(clouds, track):
    # Use central ranges per input so a smaller endpoint cloud keeps its extent.
    lows = [track.min(axis=0)]
    highs = [track.max(axis=0)]
    for cloud in clouds:
        low, high = np.quantile(cloud, (0.005, 0.995), axis=0)
        lows.append(low)
        highs.append(high)
    return np.min(lows, axis=0), np.max(highs, axis=0)


def _height_palette():
    anchors = np.array([(40, 48, 96), (34, 110, 141), (48, 157, 141),
                        (161, 177, 74), (230, 152, 50)])
    stops = np.linspace(0, 255, len(anchors))
    return np.column_stack([np.interp(np.arange(256), stops, c) for c in anchors.T]).astype(np.uint8)


def _raster(image, view):
    x0, y0, x1, y1 = view.box
    return np.array(image.crop((x0, y0, x1 + 1, y1 + 1)))


def _paint_height(image, view, xyz, levels, palette):
    pixels, visible = view.pixels(xyz)
    rgb = _raster(image, view)
    highest = np.full(rgb.shape[:2], -1, dtype=np.int16)
    np.maximum.at(highest, (pixels[:, 1], pixels[:, 0]), levels[visible])
    occupied = highest >= 0
    rgb[occupied] = palette[highest[occupied]]
    image.paste(Image.fromarray(rgb), view.box[:2])


def _overlay(image, view, start, end):
    rgb = _raster(image, view)
    mask = np.zeros(rgb.shape[:2], dtype=np.uint8)
    for bit, cloud in ((1, start), (2, end)):
        pixels, _ = view.pixels(cloud)
        mask[pixels[:, 1], pixels[:, 0]] |= bit
    for value, color in ((1, START), (2, END), (3, BOTH)):
        rgb[mask == value] = color
    image.paste(Image.fromarray(rgb), view.box[:2])


def write_views(output: Path, positions, map_points, *, start_points=None, end_points=None):
    """Write PNG previews; all input coordinates must already be in camera_init metres."""
    output = Path(output)
    track = _sample(positions, 100000, "trajectory", path=True)
    cloud = _sample(map_points, 400000, "voxel map")
    if (start_points is None) != (end_points is None):
        raise ValueError("supply both endpoint clouds or neither")
    results = {"trajectory": "trajectory.png", "pointcloud": "pointcloud.png"}
    frame_note = _label("FAST-LIO  ·  camera_init 坐标系  ·  单位为米  ·  XY 平面随初始化姿态确定",
                        "FAST-LIO  /  camera_init frame  /  metres  /  XY follows the initialization frame")
    image, views = _figure(_label("雷达轨迹", "Sensor trajectory"), frame_note,
                           track.min(axis=0), track.max(axis=0))
    for view in views:
        _trajectory(image, view, track)
    draw = ImageDraw.Draw(image)
    _path_key(draw)
    gap = np.linalg.norm(track[-1] - track[0])
    _text(draw, (750, 695), _label("首末单帧位置差", "First-to-last position difference")
          + f"   {gap:.3f} m", 20, anchor="lm")
    _text(draw, (45, 740), _label("两个方向等比例。每幅投影至少显示 1 m，便于区分厘米级抖动。",
                                 "Equal axis scales. Each projection spans at least 1 m to keep jitter in context."),
          17, MUTED)
    image.save(output / results["trajectory"])

    low, high = _cloud_bounds([cloud], track)
    image, views = _figure(_label("拼接点云", "Accumulated point cloud"), frame_note, low, high)
    palette = _height_palette()
    z_low, z_high = np.quantile(cloud[:, 2], (0.01, 0.99))
    z_span = max(z_high - z_low, 1e-9)
    levels = np.rint(255 * np.clip((cloud[:, 2] - z_low) / z_span, 0, 1)).astype(np.uint8)
    for view in views:
        _paint_height(image, view, cloud, levels, palette)
        _trajectory(image, view, track)
    image.paste(Image.fromarray(palette[None, :, :]).resize((240, 14)), (925, 687))
    draw = ImageDraw.Draw(image)
    _path_key(draw)
    _text(draw, (905, 695), f"Z   {z_low:.2f} m", 16, anchor="rm")
    _text(draw, (1180, 695), f"{z_high:.2f} m", 16, anchor="lm")
    sample_size = f"{len(cloud):,} / {len(map_points):,}"
    _text(draw, (45, 740), _label(f"预览抽取 {sample_size} 个体素点。颜色显示每像素最大的 Z。视窗取点云主要范围，完整点云见 PLY。",
                                 f"Preview sample: {sample_size} voxel points. Colour shows maximum Z per pixel. View fits the core extent. See PLY for all points."),
          17, MUTED)
    image.save(output / results["pointcloud"])

    if start_points is not None:
        start = _sample(start_points, 200000, "start cloud")
        end = _sample(end_points, 200000, "end cloud")
        low, high = _cloud_bounds([start, end], track[[0, -1]])
        image, views = _figure(_label("闭环检查 · 首尾点云叠加", "Closure inspection / endpoint overlay"),
                               frame_note, low, high)
        for view in views:
            _overlay(image, view, start, end)
        draw = ImageDraw.Draw(image)
        for x, color, label in [(45, START, _label("起始时段", "Start window")),
                                (334, END, _label("结束时段", "End window")),
                                (620, BOTH, _label("两者落入同一像素", "Both windows in the same pixel"))]:
            draw.rectangle((x, 686, x + 22, 704), fill=color)
            _text(draw, (x + 33, 695), label, 18, anchor="lm")
        _text(draw, (45, 740), _label("使用原始估计位姿，未经首尾重新配准。重复的墙边或门框可提示偏移，像素相交还会受投影方向影响。",
                                     "Windows use the original estimated poses. Look for doubled edges. Pixel overlap depends on projection."),
              17, MUTED)
        results["closure_overlay"] = "closure_overlay.png"
        image.save(output / results["closure_overlay"])
    return results


def write_viewer(output: Path, positions, map_points):
    """Embed bounded camera_init point and path samples in an offline HTML file."""
    track = _sample(positions, 16000, "trajectory", path=True)
    cloud = _sample(map_points, 65000, "voxel map")
    low, high = _cloud_bounds([cloud], track)

    def encode(xyz):
        return base64.b64encode(xyz.astype("<f4").tobytes()).decode("ascii")

    data = {
        "cloud": encode(cloud), "track": encode(track),
        "mapCount": len(map_points), "trackCount": len(positions),
        "hasOverlay": (Path(output) / "closure_overlay.png").is_file(),
        "center": ((low + high) / 2).tolist(),
        "span": max(1.0, float(np.linalg.norm(high - low)) * 1.08),
        "zRange": np.quantile(cloud[:, 2], (0.01, 0.99)).tolist(),
        "palette": _height_palette().tolist(),
    }
    template = Path(__file__).with_name("offline_viewer.html").read_text(encoding="utf-8")
    target = Path(output) / "viewer.html"
    target.write_text(template.replace("__DATA__", json.dumps(data, separators=(",", ":"))),
                      encoding="utf-8")
    return target.name
