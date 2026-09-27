#!/usr/bin/env python3
"""Bench check for the AprilTag used in camera<->drone calibration.

Detects tag36h11 in the D435i color stream and reports the tag-center position in the
color optical frame (X right, Y down, Z forward) two independent ways:

  pnp    from the tag's printed size (solvePnP, IPPE_SQUARE)
  depth  from the aligned depth image, median over a patch at the tag center

On a flat, well-lit tag the two should agree to ~1-2 cm, and both should match a tape
measure. A consistent scale error in `pnp` alone means --size is wrong (measure the
OUTER BLACK square edge, not the white border).

Run in the project venv, with nothing else holding the camera:
  python scripts/apriltag_bench.py --size 0.156 --seconds 20
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from camera import RealSenseCamera  # noqa: E402

PATCH = 5  # half-width of the depth median patch, pixels


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--size", type=float, default=0.156, help="outer black square edge, meters")
    p.add_argument("--seconds", type=float, default=20.0, help="run time")
    p.add_argument("--id", type=int, default=None, help="only report this tag id")
    return p.parse_args()


def depth_at(camera, depth_image, u, v):
    h, w = depth_image.shape
    u0, u1 = max(int(u) - PATCH, 0), min(int(u) + PATCH + 1, w)
    v0, v1 = max(int(v) - PATCH, 0), min(int(v) + PATCH + 1, h)
    patch = depth_image[v0:v1, u0:u1].astype(float)
    valid = patch[patch > 0]
    if valid.size < patch.size // 4:
        return None
    z = float(np.median(valid)) * camera.depth_scale
    import pyrealsense2 as rs
    return np.array(rs.rs2_deproject_pixel_to_point(camera.intrinsics, [float(u), float(v)], z))


def main():
    args = parse_args()
    half = args.size / 2.0
    # Corner order matches OpenCV's detector output: TL, TR, BR, BL (tag frame, z out of tag).
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )

    rows = []  # (pnp_xyz, depth_xyz or None, side_px, reproj_px, tilt_deg)
    frames = 0
    with RealSenseCamera() as camera:
        intr = camera.intrinsics
        K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
        dist = np.array(intr.coeffs, dtype=np.float64)
        print(f"color {intr.width}x{intr.height}  fx={intr.fx:.1f} fy={intr.fy:.1f}  tag size {args.size:.3f} m")

        t_end = time.time() + args.seconds
        last_print = 0.0
        while time.time() < t_end:
            color, depth = camera.get_aligned_frames()
            if color is None:
                continue
            frames += 1
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
            if ids is None:
                continue
            for c, tag_id in zip(corners, ids.flatten()):
                if args.id is not None and tag_id != args.id:
                    continue
                img = c.reshape(4, 2).astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
                reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1))))
                side = float(np.mean(np.linalg.norm(img - np.roll(img, 1, axis=0), axis=1)))
                R, _ = cv2.Rodrigues(rvec)
                tilt = float(np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1))))  # 0 = facing camera
                pnp = tvec.flatten()
                u, v = img.mean(axis=0)
                dep = depth_at(camera, depth, u, v)
                rows.append((pnp, dep, side, reproj, tilt))

                now = time.time()
                if now - last_print >= 1.0:
                    last_print = now
                    dtxt = "no depth" if dep is None else (
                        f"depth X={dep[0]:+.3f} Y={dep[1]:+.3f} Z={dep[2]:.3f}  "
                        f"|pnp-depth|={np.linalg.norm(pnp - dep) * 100:.1f} cm")
                    print(f"id {tag_id}  side {side:5.1f}px  tilt {tilt:4.1f}deg  reproj {reproj:.2f}px  "
                          f"pnp X={pnp[0]:+.3f} Y={pnp[1]:+.3f} Z={pnp[2]:.3f}  {dtxt}")

    print(f"\n{frames} frames, tag seen in {len(rows)} ({100 * len(rows) / max(frames, 1):.0f}%)")
    if not rows:
        print("No tag detected. Check lighting, distance (>= ~25 px side), and that it is tag36h11.")
        return
    pnp = np.array([r[0] for r in rows])
    deps = np.array([r[1] for r in rows if r[1] is not None])
    print(f"pnp   mean X={pnp[:, 0].mean():+.3f} Y={pnp[:, 1].mean():+.3f} Z={pnp[:, 2].mean():.3f}  "
          f"std {pnp.std(axis=0).max() * 1000:.1f} mm (worst axis)")
    if len(deps):
        print(f"depth mean X={deps[:, 0].mean():+.3f} Y={deps[:, 1].mean():+.3f} Z={deps[:, 2].mean():.3f}  "
              f"std {deps.std(axis=0).max() * 1000:.1f} mm (worst axis)")
        print(f"mean |pnp - depth| = {np.linalg.norm(pnp.mean(axis=0) - deps.mean(axis=0)) * 100:.1f} cm")
    print(f"side {np.mean([r[2] for r in rows]):.1f} px, reproj {np.mean([r[3] for r in rows]):.2f} px, "
          f"tilt {np.mean([r[4] for r in rows]):.1f} deg")


if __name__ == "__main__":
    main()
