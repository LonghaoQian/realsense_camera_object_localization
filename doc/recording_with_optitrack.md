# Recording RealSense + OptiTrack with rosbag2

Records the D435i's color/depth streams alongside its own OptiTrack pose
(camera-mounted markers, no drone), synchronized by ROS2 bag timestamps, for
offline landmark estimation later (see `slam_options.md` for background on
why this replaces onboard SLAM/VIO when mocap ground-truth pose is
available).

This is a separate, system-wide ROS2 install (`sudo apt`), not part of the
project's Python venv. As of 2026-08-20 none of the packages below are
installed on the Jetson Orin Nano (`yahboom`), which does have ROS2 Humble
itself — so step 1 is still required there before any of this works.

Note that the ROS2 path and the Python path drive the *same* camera through
different stacks: `ros-humble-realsense2-camera` wraps librealsense
independently of the venv's `pyrealsense2`. Only one process can hold the
D435i at a time, so stop `main.py` (see `jetson_orin_setup.md` for the tmux
workflow) before launching the ROS2 node, or the launch will fail with
`errno=16 Device or resource busy`.

Before a session, `preflight_stream_check.md` walks through confirming the
camera link, exposure, detections and depth in a browser — worth doing while
problems are still fixable, and it ends by freeing the camera for the steps
below.

## 0. Before you go to the lab

Three things that are cheap to fix at your desk and expensive to discover in
the lab. All three were unmet on `yahboom` as of 2026-08-20.

**Packages** — see step 1. Needs internet, which the lab network may not have.

**Free space.** Recording runs on the order of a GB per minute, and the NVMe
was at 95% (9.5 GB free) — roughly nine minutes of recording before the root
filesystem fills.

```bash
df -h /
```

Plenty is reclaimable without touching anything you need:

```bash
docker builder prune -f    # ~4.7 GB of build cache, nothing running
sudo apt clean             # ~2.2 GB of cached .debs
```

Beyond that, `docker images` holds ~36 GB, all inactive — but the two largest
are the Isaac ROS images (~27 GB and ~26 GB, heavily shared layers) which are
painful to re-pull, so prune those deliberately rather than with a blanket
`docker system prune -a`. `/var/log` was also 6.7 GB, which is *not* the
journal (only 24 MB) — worth a look with sudo if space is still tight.

**Network.** The Orin must be on the same network as the Motive PC, and this
is the one that stops the session cold. From the desk network it is not:

```bash
$ ping -c2 192.168.2.201
100% packet loss          # Orin was on SSID hs293go, 192.168.50.107/24
```

In the lab, switch to the lab network (already saved, autoconnect on):

```bash
sudo nmcli device wifi connect "FSC_Lab"
ping -c2 <motive_pc_ip>          # must succeed before step 3 is worth trying
```

Switching networks **changes the Orin's IP**, so re-check `hostname -I` before
SSHing back in — see "Finding the host's IP address" in the [README](../README.md).

## 1. Install packages (one-time)

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-vrpn-mocap
```

Pulls 8 packages. `rosbag2` itself is already installed.

## 2. Define a rigid body in Motive

Put markers on the camera and define one rigid body for it in Motive (e.g.
`d435i`). Note the exact name — it's case-sensitive and becomes the ROS topic
name.

## 3. Launch the camera + mocap bridge

Run this under tmux too — a dropped SSH connection would otherwise take the
camera and VRPN nodes down mid-session:

```bash
tmux new -s rs_record
source /opt/ros/humble/setup.bash
ros2 launch ~/source/realsense_camera_object_localization/scripts/launch_camera_and_mocap.launch.py \
  server:=<motive_pc_ip> port:=3883
```

`<motive_pc_ip>` is the Motive computer's address on the lab network (see
`fsc_autopilot_ros2/docs/indoor_experiments.md` for the address currently in
use, e.g. `192.168.2.201`). `server:=` is mandatory — the launch file declares
it with no default. Confirm it pings (step 0) before bothering with this.

This starts:
- the RealSense node, aligned color+depth at 640x480@30fps
- the `vrpn_mocap` client, which auto-publishes one `geometry_msgs/PoseStamped`
  topic per rigid body Motive is currently streaming — no need to list body
  names for this step.

## 4. Verify topics before recording

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "camera|vrpn_mocap"
```

Expect to see (exact camera topic prefix may differ slightly by wrapper
version — adjust `record_rosbag.sh` if so):

```
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/vrpn_mocap/<camera_body>/pose
```

Sanity-check pose data is live:

```bash
ros2 topic hz /vrpn_mocap/<camera_body>/pose   # expect ~Motive's streaming rate (100+ Hz typical)
ros2 topic hz /camera/camera/color/image_raw   # expect ~30 Hz
```

The two failures look different and point in different directions: **no pose
topic at all, or a silent one**, is Motive or the network (wrong `server:=`,
wrong network, rigid body not streaming, name mismatch) — not the camera.
**No camera topics** is the RealSense node, most often because something else
still holds the device (see the `errno=16` note at the top).

## 5. Record

```bash
cd ~/source/realsense_camera_object_localization/scripts
CAMERA_BODY=<camera_body> ./record_rosbag.sh my_session
```

Move the camera around the space while recording to get varied viewpoints of
the objects of interest — that viewpoint diversity is what the later
multi-view fusion step needs.

Stop with `Ctrl+C`. This produces a `my_session/` directory (sqlite3 + zstd
compressed `.mcap`/`.db3` + `metadata.yaml`).

Run this in a second window of the same tmux session as the launch (`Ctrl-b c`
to open one), so a dropped WiFi connection cannot kill the recording mid-take.
After reconnecting:

```bash
scripts/tmux_status.sh       # confirm the session survived
tmux attach -t rs_record
```

Note that `scripts/tmux_kill_all.sh` kills the tmux *server* and would take the
recording down along with everything else; use `tmux kill-session -t rs_record`
to stop just this one.

Keep an eye on free space during long takes — at ~1 GB/min it goes quickly if
you did not clear room first (step 0):

```bash
df -h /
```

## 6. Inspect the recording

```bash
ros2 bag info my_session
```

Confirms message counts and rates per topic — check the pose topic and the
image topics all have the expected counts (roughly `duration * rate`), so you
can catch a topic that silently didn't record before leaving the lab.

To play it back (e.g. to re-view in `rviz2`, or re-feed into a detection
node):

```bash
ros2 bag play my_session
```

## Coordinate-frame caveat

`vrpn_mocap` publishes poses as received from Motive/VRPN. Verify axis
convention against a known motion (e.g. move the camera body straight up and
confirm which pose axis increases) before trusting it as world-frame ENU —
don't assume it matches the ENU convention `fsc_optitrack_processor_ros2`
produces downstream in the lab's normal flight pipeline, since that repo
applies its own axis conversion that this simpler recording path skips.

## Next step

Offline, extract `(camera_pose, object_3d_point_in_camera_frame)` pairs per
frame from the bag (YOLO detection + depth lookup, as in `detector.py` /
`localization.py`) and fuse across viewpoints using the recorded OptiTrack
poses — this is the landmark-estimation step discussed separately, not yet
implemented in this project.
