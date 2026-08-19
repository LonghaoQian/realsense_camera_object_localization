# Recording RealSense + OptiTrack with rosbag2

Records the D435i's color/depth streams alongside its own OptiTrack pose
(camera-mounted markers, no drone), synchronized by ROS2 bag timestamps, for
offline landmark estimation later (see `slam_options.md` for background on
why this replaces onboard SLAM/VIO when mocap ground-truth pose is
available).

Neither package below is installed yet on this machine — this is a separate,
system-wide ROS2 install (`sudo apt`), not part of the project's Python venv.

## 1. Install packages (one-time)

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-vrpn-mocap
```

## 2. Define a rigid body in Motive

Put markers on the camera and define one rigid body for it in Motive (e.g.
`d435i`). Note the exact name — it's case-sensitive and becomes the ROS topic
name.

## 3. Launch the camera + mocap bridge

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/source/realsense_d435i_detection/scripts/launch_camera_and_mocap.launch.py \
  server:=<motive_pc_ip> port:=3883
```

`<motive_pc_ip>` is the Motive computer's address on the lab network (see
`fsc_autopilot_ros2/docs/indoor_experiments.md` for the address currently in
use, e.g. `192.168.2.201`).

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

## 5. Record

```bash
cd ~/source/realsense_d435i_detection/scripts
CAMERA_BODY=<camera_body> ./record_rosbag.sh my_session
```

Move the camera around the space while recording to get varied viewpoints of
the objects of interest — that viewpoint diversity is what the later
multi-view fusion step needs.

Stop with `Ctrl+C`. This produces a `my_session/` directory (sqlite3 + zstd
compressed `.mcap`/`.db3` + `metadata.yaml`).

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
