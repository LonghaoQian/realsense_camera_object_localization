# SLAM/VIO options for camera pose (if not using external tracking)

These provide a drift-corrected global camera pose to feed into the
object-fusion step in `localization.py`, when no external tracking system
(mocap, RTK, etc.) is available. Not currently integrated into this project.

## RTAB-Map (Real-Time Appearance-Based Mapping)

**Use when:** you want a full 3D map (point cloud / occupancy grid) alongside
pose, not just the trajectory — e.g. for navigation, obstacle avoidance, or
building a persistent map of the environment.

- Consumes RGB-D directly (RealSense is natively supported), or stereo/lidar.
- Graph-based SLAM with loop closure; outputs both pose and dense/semi-dense
  map.
- Ties in easily with ROS/Nav2 — easiest of the three to get running
  out-of-the-box with a RealSense camera.
- Heavier compute and coarser pose accuracy than VIO-specialized methods;
  optimized for mapping quality more than pose precision.

## ORB-SLAM3

**Use when:** you care primarily about accurate camera trajectory/pose
(sparse map is fine) and may or may not have IMU, and want strong loop
closure / relocalization.

- Feature-based (ORB keypoints); supports monocular, stereo, RGB-D, and
  visual-inertial modes.
- Multi-map system: can lose tracking and re-localize/merge maps rather than
  restarting.
- Sparse point-cloud map — not useful for obstacle avoidance or dense
  reconstruction, but pose accuracy is strong.
- More setup than RTAB-Map (no first-class RealSense wrapper); tuning
  feature/loop-closure parameters matters.

## VINS-Fusion

**Use when:** the camera is on a fast-moving or aggressive platform (e.g. a
drone) and you have a well-calibrated IMU — this project's D435i included.

- Tightly-coupled optimization-based visual-inertial odometry, built for
  real-time onboard state estimation on UAVs.
- Handles fast motion / motion blur / brief visual dropouts better than
  vision-only methods, because IMU bridges the gaps.
- Requires accurate camera-IMU extrinsic + time-offset calibration (typically
  via Kalibr) — accuracy is sensitive to this.
- Sparse map only, no dense reconstruction; purpose-built for pose, not
  mapping.

## Summary

| | Output | Best for | Needs IMU |
|---|---|---|---|
| RTAB-Map | Pose + dense map | Mapping/navigation | No |
| ORB-SLAM3 | Pose (sparse map) | Accurate trajectory, easy relocalization | Optional |
| VINS-Fusion | Pose (sparse map) | Fast/aggressive motion (drones) | Yes |

If an external tracking system (e.g. OptiTrack) is available, none of the
above are needed — see the note in `README.md` on using external mocap
ground-truth pose instead.
