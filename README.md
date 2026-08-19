# RealSense D435i Object Detection & Localization

RGB-D object detection and 3D localization using an Intel RealSense D435i.
Detects objects on the color stream (YOLOv8) and looks up their 3D position
in the camera frame using the aligned depth stream.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

1. Confirm the camera streams correctly:

   ```bash
   source .venv/bin/activate
   python test_camera.py
   ```

   This prints the depth scale, camera intrinsics, and the 3D point at the
   image center, and saves `test_color.png` so you can eyeball the feed.

2. Run live detection + localization:

   ```bash
   source .venv/bin/activate
   python main.py
   ```

   Detected objects are drawn on screen with their label, confidence, and
   `X, Y, Z` position in meters. Press `q` to quit.

## Coordinate frame

Positions are in the **color camera's optical frame** (depth is aligned to
color, and `rs2_deproject_pixel_to_point` returns points in the intrinsics'
frame): origin at the color lens, **X right, Y down, Z forward** (out of the
lens into the scene). This is the RealSense/OpenCV optical convention, *not*
ROS REP-103 (X-forward/Y-left/Z-up) — transform accordingly if fusing with
robot body/world frames.

## Files

- `camera.py` — `RealSenseCamera`: pipeline setup, depth/color alignment,
  intrinsics, and pixel→3D deprojection.
- `detector.py` — `ObjectDetector`: YOLOv8 wrapper producing 2D bounding
  boxes (`yolov8n.pt`, COCO classes, downloaded automatically on first run).
- `localization.py` — fuses each 2D detection's bbox center with a
  noise-robust (median-of-patch) depth reading to produce a 3D position.
- `main.py` — live loop tying it all together, with an OpenCV display window.
- `test_camera.py` — standalone camera connectivity/sanity check.
- `doc/slam_options.md` — when you'd need SLAM/VIO (RTAB-Map, ORB-SLAM3,
  VINS-Fusion) for camera pose, vs. external tracking.
- `doc/recording_with_optitrack.md` + `scripts/` — ROS2 (rosbag2) recording
  of the camera stream paired with OptiTrack rigid-body poses, for offline
  multi-viewpoint landmark estimation.
- `extrinsics.yaml` + `extrinsics.py` + `doc/camera_drone_extrinsic_calibration.md`
  — static camera-to-drone transform derived from CAD, composed with the
  camera's own factory depth->color calibration.

## Swapping the detection model

`ObjectDetector` defaults to `yolov8n.pt` (fast, general-purpose, 80 COCO
classes) running on `device="cuda"`. Pass a different Ultralytics model path
(e.g. a custom-trained `.pt`) or `device="cpu"` if needed.
