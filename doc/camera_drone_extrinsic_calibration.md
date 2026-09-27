# Camera<->drone extrinsic from CAD

The rotation can be measured instead of taken from CAD, by comparing the camera's IMU
with the Pixhawk's. See `doc/imu_to_imu_rotation_calibration.md`. CAD remains the
source for the translation.

Since the mount is rigid, this is a one-time static transform, not something
recorded/tracked continuously. `extrinsics.yaml` + `extrinsics.py` hold it.

## What to measure from CAD

`extrinsics.yaml`'s `translation`/`rotation_quaternion` should be the
transform from the camera's **depth-module reference point** to the drone
body frame (FLU: X forward, Y left, Z up) — i.e. where the depth module's
origin sits and how it's oriented, expressed in drone-frame coordinates.

Use the depth module's origin specifically (not the color lens, not the
housing exterior) as the CAD reference point: Intel publishes the D435i's
mechanical CAD/STEP model with this origin at the left imager. Anchoring to
it means the code below can compose your CAD number with the camera's own
factory-calibrated depth->color offset (queried live via `pyrealsense2`,
confirmed in testing to be ~14.8mm — matches the D435i's documented
imager-to-RGB-sensor baseline) instead of you having to find the internal
color-optical-center offset by hand.

## Usage

```python
from camera import RealSenseCamera
from extrinsics import load_drone_to_color, transform_point

with RealSenseCamera() as camera:
    drone_T_color = load_drone_to_color(camera)  # composes CAD + factory calibration

    # det.position is in the color optical frame (see localization.py)
    point_in_drone_frame = transform_point(drone_T_color, det.position)
```

## Open question: does this reach the OptiTrack frame?

`drone_T_color` gets you from the color optical frame to the **drone body**
frame — but the pose OptiTrack actually publishes is the origin Motive
assigned to the rigid body when it was created (typically the marker
centroid), which is generally a *different* point/orientation than either
the drone body frame or the depth-module origin, unless you deliberately set
it.

If you used Motive's "Translate Pivot Point" (or equivalent) to place the
rigid body's origin at a known, CAD-referenced point on the camera, that
offset can be folded into `extrinsics.yaml` the same way. Otherwise, closing
the loop to OptiTrack's world frame needs one more calibration step (e.g. a
short hand-eye pass, as discussed earlier) between the Motive rigid-body
frame and the camera's depth-module origin.

Let me know which case you're in and I'll wire up the corresponding piece.
