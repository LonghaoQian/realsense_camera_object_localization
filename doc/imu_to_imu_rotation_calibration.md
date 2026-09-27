# Camera-to-drone rotation from the two IMUs

How to measure the rotation between the D435i and the drone body by comparing the
camera's own IMU with the Pixhawk's. The result goes into `extrinsics.yaml` in place of
the CAD-guessed rotation. Translation still comes from CAD or a ruler; see "Why only
the rotation" below.

**Status:** procedure only. The two scripts it names, `scripts/record_camera_imu.py` and
`scripts/solve_imu_rotation.py`, are not written yet.

## Why this works

The camera is rigidly mounted, so the camera and the Pixhawk always rotate together.
Their gyros measure the same angular velocity, each in its own axes:

```
w_body(t) = R_body_gyro * w_gyro(t + dt)
```

Rotating the drone by hand through varied motions gives thousands of matched vector
pairs. The best-fit rotation `R_body_gyro` then comes out of a standard least-squares
fit (Kabsch/Wahba). Two properties make this cheap:

- **No clock synchronisation is needed.** The time offset `dt` is estimated from the
  data itself, by cross-correlating the rotation *magnitudes* `|w|`. Magnitudes are the
  same in every frame, so they line up before the rotation is known.
- **It lands in PX4's body frame directly.** PX4's `sensor_combined.gyro_rad` is already
  rotated into the vehicle body frame (FRD) by `SENS_BOARD_ROT`, which is the frame the
  controller uses.

## Why only the rotation

- Rotation errors grow with distance: 1° of error places a target at 2 m about 3.5 cm
  off. It is the part worth calibrating.
- Translation is barely observable from gyros, and a centimetre of error in it moves
  the target by a centimetre. A ruler or the CAD model is good enough.

## Frames and the chain

| Symbol | Frame |
|---|---|
| `body` | PX4 body frame, FRD, as in `sensor_combined` |
| `drone` | `extrinsics.yaml`'s drone frame, FLU (X forward, Y left, Z up) |
| `gyro` | D435i motion module axes, whatever librealsense reports them in |
| `depth` | D435i depth-module origin (left imager), the reference `extrinsics.yaml` uses |

`extrinsics.yaml` wants the rotation `R_drone_depth`, meaning `p_drone = R p_depth`.
It is built from three parts:

```
R_drone_depth = R_drone_body * R_body_gyro * R_gyro_depth
```

- `R_drone_body` = `diag(1, -1, -1)`, the FRD-to-FLU axis flip.
- `R_body_gyro` is the result of this calibration.
- `R_gyro_depth` is the factory transform from librealsense:
  `depth_profile.get_extrinsics_to(gyro_profile)`. librealsense stores the matrix
  column-major, so reshape it with `order="F"`, the same as `extrinsics.py` does. Going
  through this call means you never need to know the IMU's axis convention by hand.

Caveat: Intel's factory transform between the IMU and the imagers is, to our knowledge,
a design value rather than a per-unit calibration. It is a small, uncalibrated link in
the chain, and the end-to-end check in step 6 is what catches it.

---

## 1. Setup (props off)

- **Props off.** The Pixhawk is powered and the drone is handled by hand.
- **Start the Agent** so `/uav_0/fmu/out/sensor_combined` is publishing:
  `fsc_autopilot_ros2/scripts/indoor_exp/start_microxrce_agent.sh udp`
- **Free the camera.** Only one process can open the D435i, so stop detection first:
  `tmux kill-session -t rs_detect`
- **Do not arm.** Nothing here needs the motors, and the control stack does not need
  to be running.

## 2. Record, both at once, for about 2 minutes

In two tmux panes, started together:

```bash
# Pixhawk gyro
mkdir -p ~/bags && cd ~/bags
ros2 bag record -o imu_calib_$(date +%Y%m%d_%H%M%S) /uav_0/fmu/out/sensor_combined

# Camera gyro + accel, via pyrealsense2, in the project venv
cd ~/dev_ws/src/realsense_camera_object_localization
source .venv/bin/activate
python scripts/record_camera_imu.py --out ~/bags/cam_imu_$(date +%Y%m%d_%H%M%S).csv
```

`record_camera_imu.py` (to be written) should:
- enable the gyro at 200 Hz and the accel at 63 Hz,
- write each sample with the frame's timestamp and its timestamp domain,
- save `R_gyro_depth` from `get_extrinsics_to()` into the same output, so the solve
  does not need the camera attached.

Keep the recording to about 2-3 minutes. The two clocks drift slowly, and one
constant offset is only valid over a short window.

## 3. The motion

1. **10 s completely still** on the bench at the start. The solver estimates both
   gyros' biases from this window, and the BMI055's bias is not small.
2. **Rotate about each axis in turn:** roll, pitch, yaw, several back-and-forth swings
   of about ±45° each. Aim for peak rates around 100-200 °/s. Both gyros stay well
   inside their ranges at that speed.
3. **Then mixed rotations** combining axes, for about 30 s.
4. **10 s still** again at the end.

Rotations about only one axis leave the fit undetermined around that axis. At least
two non-parallel axes are required; all three make it robust. Hold the frame, not the
camera or its cable. Anything that lets the camera move relative to the frame breaks
the assumption the whole method rests on.

## 4. Solve

`scripts/solve_imu_rotation.py` (to be written), in order:

1. **Timestamps.** Pixhawk: use `sensor_combined.timestamp` (PX4 clock, µs), not the
   bag's receive time. Receive times on this setup arrive in bursts. Camera: use the
   frame timestamps.
2. **Bias.** Subtract each gyro's mean over the still windows.
3. **Resample and filter.** Resample both to a common rate (e.g. 200 Hz) and low-pass
   both identically (~20 Hz) to reduce noise.
4. **Time offset.** Cross-correlate `|w_body|` against `|w_gyro|` to find `dt`, and
   shift the camera series by it.
5. **Rotation.** Fit `R_body_gyro` with Kabsch (SVD) on the moving samples only, where
   `|w| > ~20 °/s`. Check `det(R) = +1`.
6. **Compose** `R_drone_depth` as in the chain above. Print it as an `[x, y, z, w]`
   quaternion for `extrinsics.yaml`.

## 5. Check the result before trusting it

| Check | Accept if |
|---|---|
| Fit residual, RMS of `w_body - R w_gyro` on moving samples | comparable to gyro noise, a few °/s; large means a bad offset or a loose mount |
| Split test: solve on the first and second halves separately | the two rotations agree within ~0.5° |
| Excitation: smallest singular value of the fit's 3x3 matrix | not near zero; near zero means an axis was not exercised |
| Sanity vs. the mount | close to how the camera visibly points, e.g. looking forward: within a few degrees of the CAD rotation |
| Time offset `dt` | stable across the split halves |

If a check fails, redo the recording rather than adjusting the numbers.

## 6. Finish and verify end to end

1. Write the quaternion into `extrinsics.yaml` `rotation_quaternion`.
2. Measure the translation from the Pixhawk's center to the depth-module origin (the
   left imager; see `doc/camera_drone_extrinsic_calibration.md`) in FLU with a ruler or
   CAD, and write it into `translation`.
3. **End-to-end check with OptiTrack.** Put markers on the bottle, or place it at a
   mocap-surveyed point. Compare where the drone's pose plus the detection says the
   bottle is against where OptiTrack says it is, from several drone poses. Agreement
   to a few cm confirms the whole chain, including the uncalibrated IMU-to-imager link.
   A consistent angular error points at `R_gyro_depth`. A consistent offset points at
   the translation or at the mocap rigid-body pivot. See the open question in
   `doc/camera_drone_extrinsic_calibration.md`.

## Alternatives considered

- **Mocap board calibration:** put markers on an AprilTag board and solve camera-to-body
  from static poses. It is more direct (it bypasses the IMU-to-imager link) but needs a
  marker-registered board. It is the fallback if step 6 shows an angular error.
- **Kalibr camera-IMU:** the standard tool, but it needs tightly time-synced 400 Hz IMU
  and images, and syncing PX4's clock to the Orin is the weak point here.
