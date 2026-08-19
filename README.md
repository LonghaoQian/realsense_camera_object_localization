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

**On a Jetson (Orin Nano), do not use the commands above** — the CUDA-enabled
`torch`/`torchvision`/`opencv` come from JetPack, not PyPI, and a plain venv
plus `pip install -r requirements.txt` would shadow them with a CPU-only
torch. See [doc/jetson_orin_setup.md](doc/jetson_orin_setup.md).

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

   `main.py` takes `--model`, `--device`, `--conf`, `--headless` and
   `--stream`. Without a usable display (e.g. over SSH) it falls back to
   console output on its own rather than failing.

   To watch the annotated feed from another machine, add `--stream` and open
   the printed URL (`http://<host-ip>:8080/`) in a browser.

## Finding the host's IP address

The `--stream` URL depends on the host's address on the LAN, and that address
is **not stable**: the Orin takes it by DHCP (`ipv4.method:auto`), so moving to
a different router gives it a different address — usually on a different
subnet entirely. As of 2026-08-20 it is `192.168.50.107` on SSID `hs293go`, but
don't hardcode that anywhere.

The reliable answer is that `main.py --stream` prints the correct URL on
startup, whatever the current address is. Otherwise, on the host:

```bash
hostname -I          # first address is the LAN one
ip -4 -br addr show  # per-interface, if there is more than one
```

The host's own docker bridge (`172.17.0.0/16`) shows up in that output and is
*not* reachable from other machines — ignore it.

**The Orin is WiFi-only** (`wlP1p1s0`, no ethernet). If it cannot join a
network it has no address at all and SSH is impossible — recovery then needs a
monitor and keyboard. So add a new network *before* switching to it, while the
box is still reachable:

```bash
sudo nmcli device wifi connect "<SSID>" password "<password>"
nmcli -t -f NAME,TYPE,AUTOCONNECT connection show | grep wireless   # already saved
```

Several lab networks (`FSC_Lab`, `DroneNET_5G-2`, `Yahboom3`/`4`, …) are
already saved with autoconnect, so the Orin rejoins those on its own.

### mDNS (`yahboom.local`) — currently broken

`avahi-daemon` is running and `mdns4_minimal` is in `/etc/nsswitch.conf`, so
`yahboom.local` ought to be the router-independent way in. It is not usable
yet: avahi advertises every interface, and the `DOWN` docker bridge wins.

```
$ avahi-resolve -n -4 yahboom.local
yahboom.local   172.17.0.1      # docker0, unreachable from the LAN
```

To fix, set the interface explicitly in `/etc/avahi/avahi-daemon.conf`:

```
allow-interfaces=wlP1p1s0
```

then `sudo systemctl restart avahi-daemon`. After that `ssh jetson@yahboom.local`
and `http://yahboom.local:8080/` work across any router change, and no IP
hunting is needed.

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
- `stream.py` — `MJPEGServer`: serves the annotated frames over HTTP so the
  feed can be watched in a browser on another machine (stdlib only).
- `test_camera.py` — standalone camera connectivity/sanity check.
- `doc/jetson_orin_setup.md` — Orin Nano setup (JetPack CUDA stack, aarch64
  `pyrealsense2`, measured on-device throughput, headless/display notes).
- `doc/preflight_stream_check.md` — step-by-step check of the live feed in a
  browser before a data-collection session.
- `scripts/run_detection_tmux.sh`, `scripts/tmux_status.sh`,
  `scripts/tmux_kill_all.sh` — run the detection loop in a detached tmux
  session so it survives SSH disconnects, check what's alive, tear it down.
- `scripts/reset_camera.sh` — firmware-resets the D435i over USB and verifies
  it streams again; the fix for `Frame didn't arrive within 5000` after an
  unclean stop.
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

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
