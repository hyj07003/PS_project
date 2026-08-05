from __future__ import annotations

import io
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class MapInfo:
    map_id: str
    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.25

    def to_dict(self) -> dict[str, Any]:
        return {
            "mapId": self.map_id,
            "resolution": self.resolution,
            "origin": {
                "x": self.origin_x,
                "y": self.origin_y,
                "yaw": self.origin_yaw,
            },
            "width": self.width,
            "height": self.height,
            "negate": self.negate,
            "occupiedThresh": self.occupied_thresh,
            "freeThresh": self.free_thresh,
        }

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """map frame (m) → image pixel (col, row). ROS: image row 0 is top, +y is up in map."""
        col = (x - self.origin_x) / self.resolution
        row = self.height - 1 - (y - self.origin_y) / self.resolution
        return col, row

    def pixel_to_world(self, col: float, row: float) -> tuple[float, float]:
        """image pixel (col, row) → map frame (m)."""
        x = self.origin_x + col * self.resolution
        y = self.origin_y + (self.height - 1 - row) * self.resolution
        return x, y


def _parse_yaml_simple(text: str) -> dict[str, Any]:
    """Minimal YAML subset for ROS map yaml (no PyYAML required)."""
    out: dict[str, Any] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            continue
        key, val = line.split(":", 1)
        key = key.strip()
        val = val.strip()
        if key == "origin":
            # [x, y, yaw]
            inner = val.strip("[]")
            parts = [p.strip() for p in inner.split(",")]
            out[key] = [float(parts[0]), float(parts[1]), float(parts[2])]
        elif key in ("resolution", "occupied_thresh", "free_thresh"):
            out[key] = float(val)
        elif key == "negate":
            out[key] = int(val)
        else:
            out[key] = val.strip().strip("'\"")
    return out


def _pgm_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"unsupported PGM magic: {magic!r}")
        while True:
            line = f.readline()
            if not line:
                raise ValueError("truncated PGM header")
            s = line.strip()
            if not s or s.startswith(b"#"):
                continue
            parts = s.split()
            if len(parts) >= 2:
                return int(parts[0]), int(parts[1])
            # width alone then height next
            w = int(parts[0])
            while True:
                line2 = f.readline()
                if not line2:
                    raise ValueError("truncated PGM size")
                s2 = line2.strip()
                if not s2 or s2.startswith(b"#"):
                    continue
                return w, int(s2.split()[0])


def resolve_map_paths(map_id: str | None = None) -> tuple[str, Path, Path]:
    mid = (map_id or os.environ.get("PINKY_MAP", "map_test1")).strip()
    root = Path(__file__).resolve().parents[1]
    candidates = [
        Path.cwd() / mid,
        root / mid,
    ]
    yaml_path = None
    for base in candidates:
        y = base if base.suffix == ".yaml" else Path(str(base) + ".yaml")
        if y.is_file():
            yaml_path = y
            break
        y2 = Path(str(base)).with_suffix(".yaml") if base.suffix else Path(str(base) + ".yaml")
        if y2.is_file():
            yaml_path = y2
            break
    if yaml_path is None:
        raise FileNotFoundError(f"map yaml not found for PINKY_MAP={mid}")

    data = _parse_yaml_simple(yaml_path.read_text(encoding="utf-8"))
    image_name = str(data.get("image", f"{mid}.pgm"))
    image_path = yaml_path.parent / image_name
    if not image_path.is_file():
        raise FileNotFoundError(f"map image not found: {image_path}")
    return mid, yaml_path, image_path


def load_map_info(map_id: str | None = None) -> MapInfo:
    mid, yaml_path, image_path = resolve_map_paths(map_id)
    data = _parse_yaml_simple(yaml_path.read_text(encoding="utf-8"))
    origin = data.get("origin", [0.0, 0.0, 0.0])
    width, height = _pgm_size(image_path)
    return MapInfo(
        map_id=mid,
        yaml_path=yaml_path,
        image_path=image_path,
        resolution=float(data.get("resolution", 0.05)),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
        width=width,
        height=height,
        negate=int(data.get("negate", 0)),
        occupied_thresh=float(data.get("occupied_thresh", 0.65)),
        free_thresh=float(data.get("free_thresh", 0.25)),
    )


def map_png_bytes(map_id: str | None = None) -> bytes:
    """Convert map PGM to PNG for browsers."""
    info = load_map_info(map_id)
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Pillow required for /map/image — pip install Pillow") from exc

    img = Image.open(info.image_path)
    if img.mode != "L":
        img = img.convert("L")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()
