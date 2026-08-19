"""Fuse 2D detections with the depth frame to get 3D object positions."""

from dataclasses import dataclass

import numpy as np

from detector import Detection2D


@dataclass
class Detection3D:
    label: str
    confidence: float
    bbox: tuple  # (x1, y1, x2, y2) pixels
    position: np.ndarray  # [X, Y, Z] meters, camera-optical frame (Z forward)


def _robust_depth_patch(depth_image, cx, cy, patch=5):
    """Median depth (raw units) over a small patch around (cx, cy), ignoring zero (invalid) reads."""
    h, w = depth_image.shape
    half = patch // 2
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    region = depth_image[y0:y1, x0:x1]
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def localize(camera, detections: list[Detection2D], depth_image) -> list[Detection3D]:
    results = []
    for det in detections:
        cx, cy = det.center
        depth_raw = _robust_depth_patch(depth_image, cx, cy)
        if depth_raw is None:
            continue  # no valid depth around this detection; skip rather than report a bad position

        depth_m = depth_raw * camera.depth_scale
        point = camera.intrinsics
        import pyrealsense2 as rs

        xyz = rs.rs2_deproject_pixel_to_point(point, [float(cx), float(cy)], depth_m)
        results.append(
            Detection3D(
                label=det.label,
                confidence=det.confidence,
                bbox=(det.x1, det.y1, det.x2, det.y2),
                position=np.array(xyz),
            )
        )
    return results
