#!/usr/bin/env bash
# Records the RealSense color/depth streams and the camera's OptiTrack pose
# into a rosbag2 bag.
#
# Run launch_camera_and_mocap.launch.py in another terminal first, and
# confirm topic names with `ros2 topic list` before recording -- the
# /vrpn_mocap/... name depends on CAMERA_BODY below, which must match the
# rigid-body name as defined in Motive.
#
# Usage:
#   ./record_rosbag.sh [bag_name]
#   CAMERA_BODY=d435i ./record_rosbag.sh my_session
set -euo pipefail

BAG_NAME="${1:-rs_optitrack_$(date +%Y%m%d_%H%M%S)}"
CAMERA_BODY="${CAMERA_BODY:-d435i}"

ros2 bag record \
  -o "$BAG_NAME" \
  --compression-mode file \
  --compression-format zstd \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  "/vrpn_mocap/${CAMERA_BODY}/pose"
