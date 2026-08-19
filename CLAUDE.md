# realsense_d435i_detection

RGB-D object detection and 3D localization on an Intel RealSense D435i.
YOLOv8 detects objects in the color stream; the aligned depth stream gives
each detection's 3D position.

## Environment

- Python venv at `.venv/` (not committed). Activate with `source .venv/bin/activate`.
- Deps: `pyrealsense2`, `ultralytics` (pulls `torch`), `opencv-python`, `numpy` — see `requirements.txt`.
- `yolov8n.pt` is downloaded to this folder on first run by Ultralytics; not committed.

## Commands

```bash
source .venv/bin/activate
python test_camera.py   # camera connectivity/sanity check
python main.py           # live detection + localization loop, 'q' to quit
```

## Architecture

- `camera.py` — `RealSenseCamera`: pipeline start/stop, depth-to-color
  alignment, intrinsics, `deproject_pixel()` for single-point lookups.
- `detector.py` — `ObjectDetector`: YOLOv8 wrapper, returns `Detection2D`
  (label, confidence, bbox).
- `localization.py` — `localize()`: takes `Detection2D`s + the depth frame,
  reads a median depth over a small patch around each bbox center (robust to
  single-pixel dropout/noise), deprojects to `Detection3D` (adds `position`).
- `main.py` — ties the above into a live OpenCV display loop.

## Coordinate frame

3D positions are in the **color camera's optical frame** (depth aligned to
color): X right, Y down, Z forward from the lens. This is the
RealSense/OpenCV convention, not ROS REP-103 — transform before fusing with
robot body/world frames.

## Notes

- Detector runs on `device="cuda"` by default (RTX 3080 available on this
  machine); pass `device="cpu"` to `ObjectDetector` if running without a GPU.
- Detections with no valid depth in their patch (e.g. reflective/out-of-range
  surfaces) are silently skipped in `localize()` rather than reported with a
  bad position.

## ROS2 / OptiTrack recording (separate from the Python venv above)

- `doc/recording_with_optitrack.md` + `scripts/launch_camera_and_mocap.launch.py`
  + `scripts/record_rosbag.sh` — records the RealSense color/depth streams
  and the camera's own OptiTrack pose (markers mounted directly on the
  camera) to a rosbag2 bag, for offline multi-viewpoint landmark estimation
  (fusing detections across camera poses — see `doc/slam_options.md` for why
  OptiTrack ground-truth pose is used here instead of onboard SLAM/VIO).
- Uses system ROS2 Humble (`ros-humble-realsense2-camera`,
  `ros-humble-vrpn-mocap`), not the project venv — these are the ROS2
  wrappers around the same camera, separate from `pyrealsense2`.
- This machine is not part of `~/source/fsc_autopilot_ws`'s indoor-flight
  pipeline; recording here skips that workspace's
  `fsc_optitrack_processor_ros2` (which does VRPN→ENU axis conversion) for
  simplicity — see the coordinate-frame caveat in `recording_with_optitrack.md`
  before treating recorded poses as ENU.
- Landmark-fusion code (combining recorded bag data across viewpoints into a
  refined global object position) is not yet implemented.

## Camera<->drone extrinsic (CAD-derived)

- `extrinsics.yaml` — CAD-measured static transform, depth-module reference
  point -> drone body frame (FLU). Fill in from the mounting-structure CAD.
- `extrinsics.py` — `load_drone_to_color(camera)` composes that CAD transform
  with the D435i's own factory depth->color calibration (queried live via
  `pyrealsense2`, not hardcoded) to get depth-module-CAD-point -> color
  optical frame in one 4x4 matrix; `transform_point()` applies it.
  Verified against the live device: composed color-optical-center offset
  comes out ~14.8mm from the depth origin, matching the D435i's documented
  imager baseline.
- See `doc/camera_drone_extrinsic_calibration.md` for the CAD reference-point
  convention and an open question: whether the OptiTrack rigid-body origin
  (as defined in Motive) coincides with a known CAD point on the camera, or
  needs its own calibration step to relate to `extrinsics.yaml`'s frame.
