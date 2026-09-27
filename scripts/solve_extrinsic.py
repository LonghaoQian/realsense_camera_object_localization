#!/usr/bin/env python3
"""Solve the drone-body -> camera-color-optical-frame extrinsic from the synced
AprilTag + `/uav_0/mocap` samples in extrinsic_calib_log.csv (see
apriltag_mocap_calib.py, which produces that log).

Each sample gives one 3D-point correspondence: the tag's known Motive-frame position,
transformed into that sample's drone-body frame using mocap's own reported body pose
(ENU world / FLU body), versus the tag's position as directly observed in the camera's
color optical frame (apriltag_bench-style PnP/depth). With N differently-posed samples
this is exactly the "absolute orientation" / rigid point-set registration problem, and
is solved here with Kabsch/SVD -- the same closed-form least-squares fit ICP uses for
its rigid step. No CAD numbers are used; this is a from-data extrinsic.

Remaining unknown: how the *raw Motive* tag coordinate (as read off Motive's UI) maps
into the ENU frame `/uav_0/mocap` actually publishes in. That conversion normally lives
in fsc_optitrack_processor_ros2, which runs on the ground-station laptop, not here, so
it can't be read from source on this machine. Rather than guess, this script tries a
set of physically-plausible axis conventions (Motive is conventionally Y-up; ENU is
Z-up, so the live candidates keep X and permute/flip Y<->Z) and reports the fit
residual for each -- with 8 independently-posed samples, the correct convention should
produce a clearly smaller residual than the wrong ones, since a wrong convention has no
reason to accidentally fit a consistent rigid transform across 8 diverse views.

Usage:
    python scripts/solve_extrinsic.py --tag 0.5451 0.017087 0.929056
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from extrinsics import _quat_to_matrix  # noqa: E402

CANDIDATES = {
    "identity  (x, y, z)":  lambda x, y, z: np.array([x, y, z]),
    "swap      (x, z, y)":  lambda x, y, z: np.array([x, z, y]),
    "swap+flip (x,-z, y)":  lambda x, y, z: np.array([x, -z, y]),
    "swap+flip (-x, z, y)": lambda x, y, z: np.array([-x, z, y]),
    "swap+flip (-x,-z, y)": lambda x, y, z: np.array([-x, -z, y]),
    "swap+flip (x, z,-y)":  lambda x, y, z: np.array([x, z, -y]),
    "swap+flip (x,-z,-y)":  lambda x, y, z: np.array([x, -z, -y]),
    "swap+flip (-x, z,-y)": lambda x, y, z: np.array([-x, z, -y]),
    "swap+flip (-x,-z,-y)": lambda x, y, z: np.array([-x, -z, -y]),
}


def load_rows(csv_path):
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def kabsch(P, Q):
    """Best-fit rigid R,t mapping P (Nx3) -> Q (Nx3) in the least-squares sense."""
    p_bar, q_bar = P.mean(axis=0), Q.mean(axis=0)
    Pc, Qc = P - p_bar, Q - q_bar
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = q_bar - R @ p_bar
    return R, t


def solve_for_tag(tag_world, rows, field):
    P_cam, Q_body = [], []
    for row in rows:
        R_body = _quat_to_matrix([float(row["mocap_qx"]), float(row["mocap_qy"]),
                                   float(row["mocap_qz"]), float(row["mocap_qw"])])
        t_body = np.array([float(row["mocap_x"]), float(row["mocap_y"]), float(row["mocap_z"])])
        p_cam = np.array([float(row[f"{field}_x"]), float(row[f"{field}_y"]), float(row[f"{field}_z"])])
        P_cam.append(p_cam)
        Q_body.append(R_body.T @ (tag_world - t_body))
    P_cam, Q_body = np.array(P_cam), np.array(Q_body)
    R_ext, t_ext = kabsch(P_cam, Q_body)
    residuals = np.linalg.norm((P_cam @ R_ext.T + t_ext) - Q_body, axis=1)
    return R_ext, t_ext, residuals


def rotation_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S
    return np.array([x, y, z, w])


def run(rows, tag_raw, field):
    results = []
    for name, convert in CANDIDATES.items():
        tag_world = convert(*tag_raw)
        R_ext, t_ext, residuals = solve_for_tag(tag_world, rows, field)
        rms = np.sqrt(np.mean(residuals ** 2))
        results.append((rms, name, tag_world, R_ext, t_ext, residuals))
    results.sort(key=lambda r: r[0])
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log", type=Path, default=Path(__file__).resolve().parent.parent / "extrinsic_calib_log.csv")
    p.add_argument("--tag", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                   help="AprilTag center, raw Motive-frame coordinates as read off Motive")
    args = p.parse_args()

    rows = load_rows(args.log)
    print(f"{len(rows)} samples loaded from {args.log}\n")

    for field in ("depth", "pnp"):
        print(f"=== using '{field}' camera-frame tag reading ===")
        results = run(rows, args.tag, field)
        print(f"{'convention':<24} {'ENU tag coord':<30} rms (cm)")
        for rms, name, tag_world, *_ in results:
            print(f"{name:<24} {np.array2string(tag_world, precision=4):<30} {rms * 100:6.2f}")

        best_rms, best_name, best_tag_world, R_ext, t_ext, residuals = results[0]
        q = rotation_to_quat(R_ext)
        print(f"\nbest: '{best_name}'  tag_world(ENU)={best_tag_world}  rms={best_rms*100:.2f} cm")
        print(f"per-sample residuals (cm): {np.round(residuals * 100, 1)}")
        print(f"drone_T_color  translation: [{t_ext[0]:.4f}, {t_ext[1]:.4f}, {t_ext[2]:.4f}]")
        print(f"               rotation_quaternion (x,y,z,w): "
              f"[{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]\n")


if __name__ == "__main__":
    main()
