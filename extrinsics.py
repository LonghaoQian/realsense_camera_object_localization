"""Static camera<->drone extrinsic, composed with the D435i's own factory
depth->color calibration so extrinsics.yaml only needs a CAD-measured
transform to the camera's depth-module reference point, not the internal
color-optical-center offset (queried live from the device instead).
"""

import numpy as np
import pyrealsense2 as rs
import yaml


def _quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _to_homogeneous(rotation, translation):
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def _invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def load_drone_to_depth(yaml_path="extrinsics.yaml"):
    """CAD-measured transform: depth-module reference point -> drone body frame.

    p_drone = load_drone_to_depth() @ p_depth (homogeneous)
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    rotation = _quat_to_matrix(data["rotation_quaternion"])
    return _to_homogeneous(rotation, np.array(data["translation"], dtype=float))


def load_drone_to_color(camera, yaml_path="extrinsics.yaml"):
    """Compose the CAD-measured drone<-depth transform with the device's factory
    depth<->color calibration, to get: p_drone = load_drone_to_color(camera) @ p_color
    """
    drone_T_depth = load_drone_to_depth(yaml_path)

    depth_stream = camera.profile.get_stream(rs.stream.depth)
    color_stream = camera.profile.get_stream(rs.stream.color)
    extrin = depth_stream.get_extrinsics_to(color_stream)  # depth-frame point -> color-frame point

    color_T_depth = _to_homogeneous(
        np.array(extrin.rotation).reshape(3, 3, order="F"),  # librealsense stores column-major
        np.array(extrin.translation),
    )
    depth_T_color = _invert_transform(color_T_depth)
    return drone_T_depth @ depth_T_color


def transform_point(T, point):
    """Apply a 4x4 homogeneous transform to a 3D point."""
    p = np.append(np.asarray(point, dtype=float), 1.0)
    return (T @ p)[:3]
