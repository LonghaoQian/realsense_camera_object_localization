# Fisheye webcam + pitch gimbal, for camera coverage during grasp

Context: a cable-wrench with a gripper hangs from the drone. The D435i is
fixed-mounted for far-field detection/localization, but goes blind on the
target once the drone is centered directly above it for the grab — no
mounting angle fixes this alone, since the D435i also has a hard minimum
depth range (~20-30cm). This doc records the investigation and the resulting
design decisions.

## Fisheye USB webcam

- Detected as `/dev/video0` (+ `/dev/video1`, metadata-only), vendor/model
  `KS2A543` (USB ID `2304:2334`) — separate device from the D435i's six video
  nodes (`video2`-`video7`).
- Opens fine with `cv2.VideoCapture(0, cv2.CAP_V4L2)`, up to 1920x1080.
- Genuinely fisheye/wide-angle, mounted looking down; real barrel distortion
  is visible across the frame. A `solvePnP`-based pose (as used in
  `scripts/apriltag_bench.py`) will be wrong on this lens unless it's
  calibrated (`cv2.fisheye.calibrate`) and undistorted first, or its
  distortion coefficients are supplied directly.

### AprilTag test on this camera

Ran the same `tag36h11` / `cv2.aruco` decoder `scripts/apriltag_bench.py`
uses for the D435i, against a tag on the desk ~40-60cm from the lens.

- Default auto-exposure blew out the white background around the tag,
  bleeding into the black border — quad candidates were found but couldn't
  be decoded (13 rejected).
- Manual exposure fixed the blowout, but the tag was still soft/blurry at 4x
  zoom — looks like the module's fixed-focus near limit doesn't reach this
  distance, not a lighting problem.
- Next step if this is picked back up: move the tag farther from the lens
  (or check whether this module has a focus ring) before concluding
  detection won't work on it.

### AprilTag vs YOLO for object identification

- YOLO: open-set — recognizes an object's own appearance, no marker needed.
- AprilTag: closed-set — reads a printed ID and gives a full 6-DOF pose (not
  just a bbox) for objects that have been physically tagged. More reliable
  than a neural detector for a known/instrumented object set, but blind to
  anything untagged.
- Natural split for this project: tags for a specific instrumented object
  set, YOLO+depth for general/untagged classes.

## The blind-spot problem and options considered

1. **Two-camera handoff** — RealSense for far-field search/3D localization,
   fisheye webcam (wide FOV, no depth-minimum-range limit) for the terminal
   approach once near-overhead, optionally with an AprilTag on the target
   for a robust close-in centering signal. Lowest hardware complexity; needs
   a sensor-handoff state machine.
2. **Open-loop last-known-position** — localize once with the RealSense
   while the object is still visible, convert to a world-frame waypoint via
   the drone's own pose estimate, then descend with no further visual
   feedback. Simplest, but fragile to a moving object or pose-estimate
   drift.
3. **Gimbal** — tilt the D435i itself to keep the object in view through the
   whole approach. **Chosen.**

## Gimbal design

- **Single pitch axis only**, roughly 0&deg; (level/forward, search) to
  ~90&deg; (straight down, approach). No roll/yaw axis needed — the drone's
  own yaw covers azimuth, and it stays roughly level in hover.
- **Servo: lightweight PWM, metal-gear, digital** — not analog, not plastic
  (nylon) gears. The static torque requirement is small (D435i ~72g + mount,
  short lever arm), so torque spec isn't the limiter; gear material and
  matching the drone's servo-rail voltage are what matter. Plastic gears are
  expected to strip under sustained drone vibration.
- **Position feedback: AS5600 magnetic encoder**, read independently of the
  servo's own (open-loop) commanded position — this is what makes a cheap
  PWM servo safe to use despite it having no true position readback of its
  own. I2C, address `0x36`. Needs a diametrically-magnetized magnet centered
  1-3mm above the IC on the pivot shaft — mount this concentric in the
  3D-printed pivot boss by construction; it's the one dimension that's
  fussy to get right after the fact.
- **The AS5600 is absolute within one turn** — no homing move needed at
  boot. It still needs a one-time software offset: move the pivot to the
  mechanical reference position once, read the raw angle there, store it,
  then `corrected_angle = raw - offset` from then on. Don't burn the
  AS5600's OTP zero-position registers for this — it's a one-shot,
  irreversible write; a software offset does the same job safely.
- **Servo driving**: the Jetson Orin has no clean native hobby-servo PWM.
  Plan is a PCA9685 I2C PWM breakout (default address `0x40`) — shares the
  same I2C bus as the AS5600 with no address conflict.

### Open item: `extrinsics.py` assumes a static mount

`extrinsics.yaml` / `extrinsics.py` currently compose one fixed CAD
transform. A gimbal makes depth-to-body a function of live pivot angle, and
it has to be the *encoder-read* angle, not the commanded servo angle, or
backlash/settling error becomes position error on the grasp target.
**Not yet implemented** — this is the next real code change once the gimbal
hardware exists.

## Aside: QDD (torque-controlled) actuator research

Raised out of curiosity, prompted by the CubeMars AK40-10 — not currently
planned for the pitch axis (a PWM servo is the right-sized tool there; a QDD
actuator would be overkill). More relevant, if anywhere, to the
cable-wrench/gripper end effector itself, if compliant/tension-controlled
grasping is ever wanted instead of pure position control.

What makes an AK-series actuator what it is: a BLDC motor wound for
low-speed torque density, a **low-ratio** (~6-15:1) planetary gearbox chosen
specifically to stay backdrivable (the opposite goal from a harmonic-drive
gearbox), a high-res magnetic encoder for FOC commutation, and a driver
running torque/current-mode FOC rather than simple speed control.

Two off-the-shelf build paths found:

- **Buy a purpose-built metal gearbox** (preferred — wanted metal, not
  printed): [mjbots M0.5 6x Reduction Planetary
  Gearset](https://mjbots.com/products/m05-6x-planetary-gearset) — steel,
  precision-machined, 6:1 fixed ratio, ring+sun+3 planet gears on needle
  bearings, sized to nest inside an "8108-and-larger" gimbal motor stator
  (30.0mm center distance), needs grease under load (Sta-LUBE CRC SL3330
  Moly-Graph Lithium Grease). Same gearset mjbots uses in their own
  `qdd100`/Mini-Cheetah-style servos — pairs naturally with their `moteus`
  FOC controller.
- **3D-print a cycloidal gearbox** (rejected — wanted metal): a July 2025
  budget QDD build ([Hackaday
  writeup](https://hackaday.com/2025/07/14/a-budget-quasi-direct-drive-motor-inpired-by-mits-mini-cheetah/))
  used a printed cycloidal-tooth planetary stage (less backlash-prone than
  involute when printed), a hand-wound commercial stator (Halbach magnet
  array so no iron backing was needed), and a cheap embedded FOC controller
  — ~$70 total, ~30Nm holding torque, 728g, near-zero backlash.
- **Motor**: gimbal-class BLDC outrunner, >10&Omega; phase resistance,
  11-14 pole pairs (SimpleFOC's own guidance for smooth low-speed FOC
  torque, no cogging). `iPower GBM4008-150T` confirmed in use for a
  comparable actuator build in the SimpleFOC community; size up to match
  the mjbots gearset's 8108+ bore if pairing with it.
- **Encoder note (AS5047/AS5048, if this gets built)**: also absolute
  within one turn like the AS5600, but FOC commutation additionally needs
  the *electrical* zero (rotor-pole-to-stator-phase alignment), found by
  energizing one phase and letting the free rotor snap to a known position,
  then reading the encoder there — a one-time-per-assembly calibration
  (store the offset, don't re-run the alignment move on every boot,
  especially under load). Electrical angle = pole-pairs x mechanical angle
  mod 360&deg;, so pole-pair count has to be correct in software too.
