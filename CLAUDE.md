# realsense_d435i_detection

RGB-D object detection and 3D localization on an Intel RealSense D435i.
YOLOv8 detects objects in the color stream; the aligned depth stream gives
each detection's 3D position.

## Environment

- Python venv at `.venv/` (not committed). Activate with `source .venv/bin/activate`.
- Deps: `pyrealsense2`, `ultralytics` (pulls `torch`), `opencv-python`, `numpy` — see `requirements.txt`.
- `yolov8n.pt` is downloaded to this folder on first run by Ultralytics; not committed.
- **This repo runs on two different machines.** On the x86 desktop the venv is
  built the ordinary way. On the Jetson Orin Nano (`yahboom`) the venv must be
  created with `--system-site-packages` and only `pyrealsense2` installed into
  it — JetPack supplies the sole CUDA-capable `torch`/`torchvision`/`opencv`
  builds for aarch64, and `pip install -r requirements.txt` would shadow them
  with a CPU-only torch. See `doc/jetson_orin_setup.md`.

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
- `stream.py` — `MJPEGServer`: stdlib-only HTTP server publishing the
  annotated frames as MJPEG (`--stream [PORT]`), so the feed is viewable in a
  browser from another machine. Encodes only while a client is connected.
- `ros_publisher.py` — `VisionPublisher`: optional ROS2 publishing (`--ros`)
  of detections (JSON), the best detection as `PoseStamped`, and the RGB
  frame as a JPEG `CompressedImage`, so a run can be `ros2 bag record`ed
  without turning this into a colcon package. Lazily imported — untouched
  unless `--ros` is passed.
- `main.py` — ties the above into a live OpenCV display loop.

## Coordinate frame

3D positions are in the **color camera's optical frame** (depth aligned to
color): X right, Y down, Z forward from the lens. This is the
RealSense/OpenCV convention, not ROS REP-103 — transform before fusing with
robot body/world frames.

## Notes

- Detector runs on `device="cuda"` by default — RTX 3080 on the x86 desktop,
  the integrated Orin GPU on the Jetson (~31 ms/frame for `yolov8n`, giving a
  ~24 fps end-to-end loop). Pass `device="cpu"` / `--device cpu` if running
  without a GPU.
- On the Orin, watching the feed remotely is done with `--stream` (MJPEG in a
  browser), *not* X11 forwarding — see `doc/jetson_orin_setup.md`. It binds
  0.0.0.0 with no auth. `doc/preflight_stream_check.md` is the before-a-session
  runbook for that check.
- Only one process can hold the D435i at a time: a running `main.py` (easy to
  forget when detached under tmux) makes the ROS2 recording node fail with
  `errno=16 Device or resource busy`.
- `RuntimeError: Frame didn't arrive within 5000` means the camera is in a
  stuck streaming state after an unclean stop (SIGKILL, `tmux kill-server`,
  crash mid-stream) — it still enumerates on USB 3.2 and the pipeline still
  "starts", so the USB link is a red herring. Re-running does not clear it;
  `scripts/reset_camera.sh` does (firmware reset + verifies frames flow).
- `main.py` takes `--model/--device/--conf/--classes/--list-classes/--headless/--stream`,
  plus the `--ros*` family (`--ros/--ros-ns/--ros-no-image/--ros-image-hz/--ros-jpeg-quality`).
  It probes for a usable OpenCV GUI backend at startup and falls back to console-only output
  if there isn't one — JetPack's OpenCV is GTK2 and cannot open a window over forwarded
  X11, so on the Orin it is effectively always headless unless run with `DISPLAY=:0` on the
  attached monitor.
- `--classes` filters inside the model (cheaper than detecting all 80 and discarding 79); an
  unknown class name raises rather than being silently ignored. `--list-classes` prints the
  model's class names and exits.
- `--ros` publishes `<ns>/detections`, `<ns>/target/pose`, `<ns>/color/compressed`; default
  namespace is `/vision`, override with `--ros-ns` to match a vehicle in a multi-drone setup
  (e.g. `--ros-ns uav_0`). Settled recording default (commit `65d2f88`): `--ros-jpeg-quality
  80 --ros-image-hz 10` — quality barely affects encode time, but the DDS write cost tracks
  message bytes, so this keeps detections/pose at full loop rate at ~1 GB/hour. Full
  measurement in `ros_publisher.py`'s module docstring.
- `scripts/run_detection_tmux.sh` / `tmux_status.sh` / `tmux_kill_all.sh` run
  the loop detached under tmux (it outlives the SSH session), report what is
  alive, and tear everything down. Logs land in `logs/` (gitignored).
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

## To-do: getting to a first OptiTrack recording

State as of 2026-08-20 on the Orin (`yahboom`). Full procedure with commands
is in `doc/recording_with_optitrack.md`; this is the checklist of what is
actually outstanding.

**Blockers — do at the desk, need internet:**

- [ ] Install the ROS2 packages (8 new): `sudo apt install
      ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
      ros-humble-vrpn-mocap`. None are present; `rosbag2` itself already is.
- [ ] Free disk. 9.5 GB free / 95% used, and recording runs ~1 GB/min — about
      nine minutes of headroom. Safe reclaims: `docker builder prune -f`
      (~4.7 GB) and `sudo apt clean` (~2.2 GB). ~36 GB more sits in inactive
      docker images, but the two largest are the Isaac ROS images (~27 GB,
      ~26 GB) — prune those deliberately, not with a blanket
      `docker system prune -a`. `/var/log` is also 6.7 GB and is *not* the
      journal (24 MB); needs sudo to investigate.
- [ ] Desk dry-run: launch `realsense2_camera` alone (no Motive needed),
      confirm the topics publish, record ~10 s, and check `ros2 bag info`
      shows real message counts. Validates the packages, the exact topic
      names (they shift between wrapper versions — `record_rosbag.sh` hardcodes
      them), compression, and disk rate. Leaves only the VRPN half untested.

**Lab session — in order:**

- [ ] Join the lab network (`FSC_Lab`, already saved) and confirm
      `ping <motive_pc_ip>` succeeds. From the desk network the Motive PC
      (`192.168.2.201` per `fsc_autopilot_ros2/docs/indoor_experiments.md`) is
      unreachable — the Orin sits on `192.168.50.0/24`. Switching networks
      changes the Orin's IP.
- [ ] Define the camera rigid body in Motive; note the exact (case-sensitive)
      name — it becomes `CAMERA_BODY` and the topic name.
- [ ] Pre-flight the camera in the browser (`doc/preflight_stream_check.md`),
      then **stop `main.py`** — it holds the D435i and the ROS2 node cannot
      open it otherwise.
- [ ] `ros2 launch .../launch_camera_and_mocap.launch.py server:=<motive_ip>`
      under tmux. This is what starts the camera and VRPN nodes.
- [ ] Verify topics publish (`ros2 topic hz`) **before** recording.
      `record_rosbag.sh` only subscribes to six fixed topics — it starts
      nothing, and on missing topics it records an empty bag without failing.
- [ ] `CAMERA_BODY=<body> ./record_rosbag.sh <session>`, moving the camera for
      viewpoint diversity. Optionally start `scripts/system_monitor_node.py`
      first, or `/system_monitor/stats` records empty.
- [ ] `ros2 bag info <session>` before leaving the lab — a topic that silently
      did not record is cheap to catch there, expensive to catch at the desk.

**Other open items:**

- [ ] Landmark fusion across viewpoints — not implemented (see the ROS2
      section above). Decide offline-over-bag vs. a live `rclpy` detection
      node; offline is the better default, since the Orin does less during
      collection and the model can be swapped without re-collecting.
- [ ] Fill in `extrinsics.yaml` from the mounting-structure CAD, and settle
      whether the OptiTrack rigid-body origin coincides with a known CAD point
      (see the extrinsic section below).
- [ ] Optional: fix mDNS so `yahboom.local` works across router changes —
      avahi currently advertises the `DOWN` docker bridge (`172.17.0.1`).
      Set `allow-interfaces=wlP1p1s0` in `/etc/avahi/avahi-daemon.conf`. See
      the README's "Finding the host's IP address".

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
