#!/usr/bin/env python3
"""Capture one synced calibration sample: the AprilTag's position in the color optical
frame (PnP + depth, same method as apriltag_bench.py) plus the drone body's pose from
`/uav_0/mocap` at the same moment, appended as a row to a CSV log.

Run once per physical drone position, in the project venv (needs pyrealsense2/cv2) with
the system ROS2 environment available for `ros2 topic echo` (this script sources
/opt/ros/humble/setup.bash itself via subprocess, so the calling shell need not have it
sourced):

    source .venv/bin/activate
    python scripts/apriltag_mocap_calib.py --size 0.156 --label pos7

Each run holds the camera only for --seconds, then releases it. `/uav_0/mocap` position
is ENU world frame / FLU body frame (see fsc_autopilot_ros2's CLAUDE.md: the downstream
state estimator converts *from* this to NED/FRD for PX4, so this topic itself predates
that conversion). Reconciling that against a Motive-read tag coordinate (raw Motive
convention, typically Y-up) is a separate step -- this script only logs, it does not
solve the extrinsic.
"""

import argparse
import csv
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from camera import RealSenseCamera  # noqa: E402

PATCH = 5  # half-width of the depth median patch, pixels
DEFAULT_LOG = Path(__file__).resolve().parent.parent / "extrinsic_calib_log.csv"
FIELDS = [
    "timestamp", "label",
    "pnp_x", "pnp_y", "pnp_z",
    "depth_x", "depth_y", "depth_z",
    "side_px", "reproj_px", "tilt_deg", "detect_rate",
    "mocap_x", "mocap_y", "mocap_z",
    "mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw",
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--size", type=float, default=0.156, help="outer black square edge, meters")
    p.add_argument("--seconds", type=float, default=8.0, help="camera capture window")
    p.add_argument("--label", required=True, help="position label, e.g. pos7")
    p.add_argument("--id", type=int, default=None, help="only accept this tag id")
    p.add_argument("--topic", default="/uav_0/mocap", help="drone body pose topic")
    p.add_argument("--mocap-samples", type=int, default=5, help="pose reads to average")
    p.add_argument("--out", type=Path, default=DEFAULT_LOG, help="CSV log path")
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


def capture_tag(size, seconds, want_id):
    half = size / 2.0
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )

    pnp_rows, depth_rows, sides, tilts, reprojs = [], [], [], [], []
    frames = 0
    with RealSenseCamera() as camera:
        intr = camera.intrinsics
        K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
        dist = np.array(intr.coeffs, dtype=np.float64)
        print(f"color {intr.width}x{intr.height}  fx={intr.fx:.1f} fy={intr.fy:.1f}  tag size {size:.3f} m")

        t_end = time.time() + seconds
        while time.time() < t_end:
            color, depth = camera.get_aligned_frames()
            if color is None:
                continue
            frames += 1
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
            if ids is None:
                continue
            for c, tag_id in zip(corners, ids.flatten()):
                if want_id is not None and tag_id != want_id:
                    continue
                img = c.reshape(4, 2).astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
                reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1))))
                side = float(np.mean(np.linalg.norm(img - np.roll(img, 1, axis=0), axis=1)))
                R, _ = cv2.Rodrigues(rvec)
                tilt = float(np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1))))
                pnp = tvec.flatten()
                u, v = img.mean(axis=0)
                dep = depth_at(camera, depth, u, v)
                pnp_rows.append(pnp)
                if dep is not None:
                    depth_rows.append(dep)
                sides.append(side)
                tilts.append(tilt)
                reprojs.append(reproj)

    if not pnp_rows:
        return None
    pnp_rows = np.array(pnp_rows)
    return {
        "pnp": pnp_rows.mean(axis=0),
        "depth": np.array(depth_rows).mean(axis=0) if depth_rows else np.full(3, np.nan),
        "side_px": float(np.mean(sides)),
        "reproj_px": float(np.mean(reprojs)),
        "tilt_deg": float(np.mean(tilts)),
        "detect_rate": len(pnp_rows) / max(frames, 1),
    }


def read_mocap_pose(topic, samples):
    positions, quats = [], []
    for _ in range(samples):
        proc = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
             f"ros2 topic echo {topic} --once --qos-reliability reliable"],
            capture_output=True, text=True, timeout=10,
        )
        if proc.returncode != 0 or not proc.stdout.strip():
            raise RuntimeError(f"no message on {topic}: {proc.stderr.strip()}")
        msg = next(yaml.safe_load_all(proc.stdout))
        p, q = msg["pose"]["position"], msg["pose"]["orientation"]
        positions.append([p["x"], p["y"], p["z"]])
        quats.append([q["x"], q["y"], q["z"], q["w"]])
    positions = np.array(positions)
    quats = np.array(quats)
    pos_std = positions.std(axis=0).max()
    if pos_std > 0.01:
        print(f"warning: mocap position samples spread {pos_std * 1000:.1f} mm -- drone may not be static")
    return positions.mean(axis=0), quats[-1]  # last-sample quaternion; body was assumed static


def main():
    args = parse_args()
    tag = capture_tag(args.size, args.seconds, args.id)
    if tag is None:
        print("No tag detected. Check framing/lighting -- nothing logged.")
        return 1
    print(f"tag  pnp={tag['pnp']}  depth={tag['depth']}  "
          f"side={tag['side_px']:.1f}px  reproj={tag['reproj_px']:.2f}px  tilt={tag['tilt_deg']:.1f}deg  "
          f"detect_rate={tag['detect_rate']*100:.0f}%")

    mocap_pos, mocap_quat = read_mocap_pose(args.topic, args.mocap_samples)
    print(f"mocap  pos={mocap_pos}  quat_xyzw={mocap_quat}")

    new_file = not args.out.exists()
    with open(args.out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new_file:
            w.writeheader()
        w.writerow({
            "timestamp": time.time(), "label": args.label,
            "pnp_x": tag["pnp"][0], "pnp_y": tag["pnp"][1], "pnp_z": tag["pnp"][2],
            "depth_x": tag["depth"][0], "depth_y": tag["depth"][1], "depth_z": tag["depth"][2],
            "side_px": tag["side_px"], "reproj_px": tag["reproj_px"],
            "tilt_deg": tag["tilt_deg"], "detect_rate": tag["detect_rate"],
            "mocap_x": mocap_pos[0], "mocap_y": mocap_pos[1], "mocap_z": mocap_pos[2],
            "mocap_qx": mocap_quat[0], "mocap_qy": mocap_quat[1],
            "mocap_qz": mocap_quat[2], "mocap_qw": mocap_quat[3],
        })
    print(f"appended '{args.label}' to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
