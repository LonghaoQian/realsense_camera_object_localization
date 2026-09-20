# Running on the Jetson Orin Nano

Setup notes for the Orin Nano Super dev kit (`yahboom`), which differs from an
x86 desktop in two ways that matter: the CUDA PyTorch stack ships with JetPack
rather than PyPI, and the board is normally driven headless over SSH.

Verified configuration:

| Component | Version |
| --- | --- |
| Board | Jetson Orin Nano (Super) Engineering Reference Dev Kit, `MAXN_SUPER` power mode |
| L4T / JetPack | R36.4.3 (JetPack 6.2), aarch64 |
| Python | 3.10.12 (system) |
| torch / torchvision | `2.5.0a0+872d972e41.nv24.08` / `0.20.0a0+afc54f7` (JetPack builds, CUDA 12.6 + cuDNN 9.3) |
| OpenCV | 4.10.0, CUDA-enabled, GTK2 GUI |
| ultralytics | 8.3.59 |
| pyrealsense2 | 2.58.3.10794 (official `manylinux2014_aarch64` wheel) |
| Camera | D435i, serial 244622072041, FW 5.13.0.55, USB 3.2 |

## Setup

**Create the venv with `--system-site-packages`.** This is the one non-obvious
step. JetPack's `torch`/`torchvision`/`opencv` are installed into the system
`dist-packages` and are the *only* CUDA-enabled builds for this board — a plain
`python3 -m venv` isolates them away, and `pip install -r requirements.txt`
would then pull a CPU-only aarch64 torch from PyPI, silently costing ~10x
inference speed.

```bash
cd ~/source/realsense_camera_object_localization
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install pyrealsense2      # the only dep not already provided by JetPack
```

`ultralytics`, `opencv-python`, `numpy` and `pyyaml` in `requirements.txt` are
already satisfied by the system packages — do **not** `pip install -r
requirements.txt` here, as pip would replace the JetPack builds.

Confirm the stack is intact:

```bash
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# 2.5.0a0+872d972e41.nv24.08 True
```

## No udev rules or kernel patches needed

The aarch64 `pyrealsense2` wheel talks to the camera through the stock
`uvcvideo` driver, so the usual librealsense-on-Jetson dance (building from
source with `-DFORCE_RSUSB_BACKEND=true`, installing
`99-realsense-libusb.rules`, patching the kernel) does not apply. The `jetson`
user is already in the `video` group, which is all the access `/dev/video*`
requires.

Plug the camera into one of the **USB 3** ports (the two stacked ports on the
carrier board, bus 02). `lsusb -t` should show the camera at `5000M`; a `480M`
link means USB 2, which cannot sustain 640x480 color + depth at 30 fps.

## Measured performance

640x480 @ 30 fps, depth aligned to color, `yolov8n.pt` on the Orin GPU:

| Stage | Rate |
| --- | --- |
| Camera + depth-to-color alignment alone | 30.0 fps (camera-limited) |
| YOLOv8n inference alone | 30.8 ms/frame (~32 fps) |
| Full detect + localize loop | ~24 fps |

Detection is the bottleneck. If you need the full 30 fps, export the model to
TensorRT once (`yolo export model=yolov8n.pt format=engine half=True`, ~10 min
on-device) and pass the resulting `yolov8n.engine` as `--model`; expect roughly
a 3x inference speedup. The FP16 engine is device- and TensorRT-version
specific, so it must be built on the Orin itself and is not portable.

## Display

JetPack's OpenCV is built against **GTK2**, which fails to initialize over an
SSH-forwarded X connection:

```
cv2.error: ... Can't initialize GTK backend in function 'cvInitSystem'
```

`main.py` detects this and falls back to console-only output, printing each
detection's label, confidence and XYZ. To force it, pass `--headless`.

To get the live annotated window on the Orin's own monitor, run it against the
local display instead:

```bash
DISPLAY=:0 python main.py
```

## Watching the feed from another machine

`--stream` serves the annotated frames as MJPEG over HTTP, which is the
practical way to see the detections from a laptop — X11 forwarding is broken
here (above) and uncompressed X11 video over WiFi would be unusable anyway.

```bash
python main.py --stream          # port 8080
python main.py --stream 9000     # or pick your own
```

It prints the URL to open (e.g. `http://192.168.50.107:8080/`). The page is
just an `<img>` pointing at `/stream.mjpg`; any browser handles it with
nothing installed on the laptop. Multiple viewers can connect at once, and
frames are JPEG-encoded **only while at least one client is connected**, so
leaving `--stream` on in a long headless run costs nothing when nobody is
watching.

Measured: two simultaneous viewers, ~30 fps each, ~800 KB/s per viewer at the
default quality 80, with no impact on the detection loop.

This binds `0.0.0.0` and has no authentication — anyone on the LAN who knows
the port can watch the camera. Fine on a lab network, worth remembering
anywhere else.

Combined with the tmux launcher, the usual headless setup is:

```bash
scripts/run_detection_tmux.sh --stream
```

## Usage

```bash
source .venv/bin/activate
python test_camera.py                      # sanity check; writes test_color.png
python main.py                             # auto-detects display, falls back to headless
python main.py --headless --conf 0.4       # console output only
python main.py --device cpu                # if the GPU is busy with something else
DISPLAY=:0 python main.py                  # live window on the attached monitor
```

## Running detached (tmux)

Because the Orin is normally driven over SSH, a plain `python main.py` dies
with the connection. `scripts/` has three helpers for running it detached:

```bash
scripts/run_detection_tmux.sh              # start main.py --headless, detached
scripts/run_detection_tmux.sh --conf 0.4   # extra args go straight to main.py
scripts/tmux_status.sh                     # what is alive, and what is it running?
scripts/tmux_kill_all.sh                   # kill every tmux session (prompts first)
```

`run_detection_tmux.sh` starts session `rs_detect` (override with `SESSION=`),
refuses to start a second copy over a running one, and tees output to a
timestamped file under `logs/` (gitignored). The window is set `remain-on-exit`,
so if the loop crashes the pane stays around with the traceback on screen
instead of the session vanishing.

`tmux_status.sh` exits 0 only if a session is alive *and* still has a live
pane — a crashed run whose empty session lingers exits 1 — so it can gate a
restart:

```bash
scripts/tmux_status.sh rs_detect || scripts/run_detection_tmux.sh
```

`tmux_kill_all.sh` kills the whole tmux *server*, so it takes down every
session on the board, not just this project's — including a rosbag recording
if one is running under tmux. It prints what it is about to kill and asks
first; `-y` skips the prompt. To stop only the detection loop, use
`tmux kill-session -t rs_detect`.

## Harmless noise

Two messages show up on every run and can be ignored:

- `libEGL warning: DRI3/DRI2: failed to authenticate` — EGL probing over SSH.
- `nvbufsurftransform: Could not get EGL display connection` — JetPack OpenCV's
  NVIDIA buffer-transform backend, likewise probing for a display it won't find
  headless. Image encode/decode falls back to CPU and still works.

## Note on `test_camera.py`

`test_camera.py` grabs the *first* frame after starting the pipeline, before
auto-exposure and the depth pipeline have settled, so `3D point at image
center` frequently prints `None` on this board. That is not a fault — after
~30 warm-up frames roughly 78% of depth pixels are valid. `main.py` streams
continuously and is unaffected.

---

# This box (`orin0`) — a second Orin, and what differs from the table above

Everything above describes `yahboom` (user `jetson`, repo at `~/source/...`). This
checkout lives on a **different** Orin Nano — `orin0`, at
`~/dev_ws/src/realsense_camera_object_localization` — which doubles as the **flight
companion computer** running the `fsc_autopilot_ros2` control stack. Set up and verified
2026-09-11. Three things differ materially, and the first one will waste an afternoon if
you don't know it.

## 1. `LD_LIBRARY_PATH` silently binds this venv's torch to the WRONG libtorch

`~/.bashrc` line 125 prepends another environment's libtorch to the library path:

```bash
export LD_LIBRARY_PATH=/home/orin0/envs/torch_env/lib/python3.10/site-packages/torch/lib:$LD_LIBRARY_PATH
```

That is deliberate and must stay — it exists for the `torch_model_loader` ROS2 C++
package, which links libtorch **2.5.0a0+872d972e41.nv24.08** from that env. But
`LD_LIBRARY_PATH` is searched **before** a library's own `RUNPATH`, so the dynamic linker
binds this venv's torch *Python* sources against `torch_env`'s *older*
`libtorch_python.so`. The symptom looks nothing like a path problem:

```
AttributeError: module 'torch._C' has no attribute 'AcceleratorError'
```

Diagnose it with `ldd` on the extension — if `libtorch_python.so` resolves outside this
venv, that is the bug:

```bash
ldd .venv/lib/python3.10/site-packages/torch/_C.cpython-310-aarch64-linux-gnu.so | grep libtorch
```

**Already fixed:** `.venv/bin/activate` has a guarded block appended that prepends this
venv's own `torch/lib`. So a plain `source .venv/bin/activate` is correct and needs no
extra steps. If you ever rebuild the venv from scratch, re-add that block or the error
comes back.

## 2. JetPack's ML stack is NOT installed here, so the setup above is insufficient

On `yahboom`, `torch`/`torchvision`/`opencv`/`ultralytics` come from JetPack and the only
missing piece is `pyrealsense2`. **Not so here.** On `orin0`:

- `nvidia-jetpack` is **not** installed (it is available: `apt-cache policy nvidia-jetpack`
  offers `6.2.1+b38`).
- There is no system `torch`, `torchvision` or `ultralytics` at all.
- System `opencv` is stock Ubuntu **4.5.4**, *not* JetPack's CUDA-enabled 4.10. The
  README's warning about pip shadowing a CUDA OpenCV therefore does not apply here —
  there is no CUDA OpenCV to protect, so `opencv-python` from pip is fine.
- CUDA 12.6, cuDNN 9.3, TensorRT 10.3 and `libcusparseLt` **are** present, which is what
  the CUDA torch wheels actually need.

Verified working stack, installed into `.venv` (created with `--system-site-packages`):

| Package | Version | Source |
| --- | --- | --- |
| torch | 2.8.0 | jetson-ai-lab `jp6/cu126`, CUDA build |
| torchvision | 0.23.0 | jetson-ai-lab `jp6/cu126`, CUDA build (matched pair) |
| ultralytics | 8.4.148 | PyPI |
| opencv-python | 4.10.0.84 | PyPI, pinned |
| numpy | 1.26.4 | PyPI, pinned |
| pyrealsense2 | 2.58.4.10922 | PyPI `manylinux2014_aarch64` |

Confirmed: `torch.cuda.is_available()` → `True`, device `Orin`, and
`torchvision.ops.nms` runs on CUDA tensors.

### Three traps in getting those wheels

- **`pip install --extra-index-url <jetson> torch==2.8.0` gives you a CPU-only torch.**
  PyPI publishes `2.8.0+cpu`, and a local version suffix sorts *higher* than plain
  `2.8.0`, so pip prefers PyPI's CPU wheel over the jetson CUDA one. This is exactly the
  silent ~10x slowdown the README warns about. **Pin the wheel by full URL** (see
  `https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/torch/`) and pass `--no-deps`.
- **torch 2.9.x needs `libcudss.so.0`, which is absent here.** It is obtainable
  (`nvidia-cudss-cu12` on PyPI has an aarch64 wheel, or `libcudss0-cuda-12` via apt) but
  2.8.0 avoids the dependency entirely, which is why 2.8.0 is pinned.
- **Pin `opencv-python` and `numpy` explicitly.** Left alone, `pip install ultralytics`
  pulled `opencv-python` **5.0.0.93** (an unreleased major) and `numpy` 2.2.6. Both work
  in isolation but are needless risk next to a C-extension like `pyrealsense2`.

Do **not** run `pip install -r requirements.txt` here either — it would fetch a CPU-only
torch through `ultralytics`' dependency chain.

## 3. The camera is currently on a USB 2 link — and it still sustains 30 fps

`camera_info.usb_type_descriptor` reports `2.1`, and `lsusb -t` shows the D435i at
**480M** on bus 01. Bus 02 is the 10000M (USB 3.2) root hub and is empty. So the camera
is in the wrong port or behind a USB 2 hub.

Contrary to the warning earlier in this doc, that does **not** break the default config.
Measured on this link: **29.97 fps** at 640x480 aligned depth+color, inter-frame gap
33.4 ms mean / 34.0 ms max — essentially perfect. 640x480@30 is about 295 Mbps of the
480 Mbps link, so it fits, but with no headroom: higher resolutions are capped
(1280x720 colour tops out at 15 fps, depth at 6 fps). Move it to a bus-02 port if you
need more than 640x480@30.

One real symptom to not misread: **`test_camera.py` and short runs reported only ~12.5%
valid depth pixels**, against ~78% noted earlier in this doc. That was the camera lying
loose on a bench pointing at a near blank surface, not a fault — aim it at a normal scene
at 0.5-4 m before judging depth quality.

## 4. Vision must stay off the isolated CPUs

`orin0` flies the drone, so its CPUs are partitioned: the `fsc_autopilot_ros2` control
node owns isolated **CPU 5** and `MicroXRCEAgent` owns isolated **CPU 4**. Full reference:
`~/dev_ws/src/fsc_autopilot_ros2/docs/realtime_cpu_isolation.md`.

Vision is confined to the housekeeping cores **0-3** and gets **no** real-time priority —
it is throughput-bound, and a stalled frame pipeline must never be able to preempt the
control loop. `scripts/run_detection_tmux.sh` applies this itself via `taskset`; override
with `FSC_VISION_CPUS` only if the partition changes. It warns if the kernel reports no
isolated CPUs, because pinning alone does not protect anything in that case.

### Measured coexistence (2026-09-11)

With this detection loop running continuously at ~92% of one core, the autopilot's 250 Hz
control loop (4000 µs period) on isolated CPU 5 measured:

| | mean | p99 | p99.9 | **max** | missed deadlines |
|---|---|---|---|---|---|
| **control loop, vision running** | 3.6 µs | 4.4 µs | 5.0 µs | **9.7 µs** | **0 / 11250** |
| control loop, before CPU isolation, box loaded | 71 µs | 83 µs | 3630 µs | **7871 µs** | 2 / 5000 |

So the vision stack costs the flight controller **9.7 µs of worst-case jitter out of a
4000 µs budget (0.24%)**, reproduced across two separate runs. Re-measure with
`fsc_autopilot_ros2/scripts/realtime/measure_jitter.sh --seconds 60` while this loop runs,
rather than assuming it still holds after changes.

GPU inference measured **31.4 ms/frame** for `yolov8n.pt` at 640x480 — within 2% of the
30.8 ms recorded for `yahboom` above, so the CUDA path is performing as expected. Note
this is why GPU inference matters here beyond raw speed: it keeps YOLO off the CPUs,
leaving the housekeeping set free. A CPU-only torch would saturate cores 0-3 continuously.

### One honest caveat

CPU isolation does not partition **memory bandwidth, the GPU, or thermals**. The numbers
above already include a real CUDA vision workload, so they account for normal GPU use —
but a much heavier GPU/EMC load (TensorRT at high resolution, multiple models) is outside
what was measured. Re-measure if the vision workload grows substantially.

---

# Class filtering and ROS 2 recording (added 2026-09-20)

## Detecting one class only

```bash
python main.py --headless --classes bottle --conf 0.4
python main.py --list-classes            # the model's 80 COCO names, with ids
```

`--classes` is passed down to Ultralytics as `classes=[...]` rather than filtering the
results afterwards, so the model skips non-matching classes during NMS — a single-class
run is cheaper than detecting all 80 and discarding 79. Nothing else ever reaches the
console, the ROS topics, or the display.

An unknown class name **raises** instead of being ignored. A silently-dropped bad name
would leave the loop running forever reporting nothing, which looks exactly like a camera
or detection fault; failing at startup is far cheaper to diagnose.

Verified live 2026-09-20 with a water bottle at ~1.1 m: `bottle (0.54-0.69)` at
`X=-0.00 Y=0.07 Z=1.10 m`, stable, with no other objects reported.

## Publishing to ROS 2 so runs can be bagged

`main.py` is standalone and publishes nothing by default. `--ros` adds publication, with
no package/build needed — this project has no `package.xml` and is not a colcon package;
`rclpy` works from a plain script once `/opt/ros/humble/setup.bash` is sourced.

```bash
source /opt/ros/humble/setup.bash
python main.py --headless --classes bottle --conf 0.4 --ros
```

| topic | type | notes |
|---|---|---|
| `/vision/detections` | `std_msgs/String` | JSON: label, confidence, bbox, XYZ, stamp. Self-describing, survives adding fields. |
| `/vision/target/pose` | `geometry_msgs/PoseStamped` | Highest-confidence detection, for direct downstream consumption. |
| `/vision/color/compressed` | `sensor_msgs/CompressedImage` | JPEG. `--ros-no-image` omits it, `--ros-image-every N` decimates it. |

Standard message types only, because a custom `Detection3D` would require making this a
package, and `vision_msgs` (which has exactly the right type) is not installed here.

**Nothing is published on `target/pose` when there is no detection.** An empty
`PoseStamped` is indistinguishable from "target at the camera origin" — a dangerous
ambiguity for any controller consuming it. Absence of the message is the signal.

### Compressed, not raw — this is what makes recording viable

| stream | rate |
|---|---|
| raw `sensor_msgs/Image` 640x480x3 @30 | **27.6 MB/s** = 221 Mbit/s = 99 GB/hour |
| JPEG `CompressedImage`, real scene | **0.54 MB/s** = 4.3 Mbit/s = **1.9 GB/hour** |

Raw is a non-starter: it saturates WiFi outright and costs ~100 GB/hour locally, plus the
memory-bandwidth cost of moving it — the one resource CPU isolation does *not* partition.

## Where to record: on the Orin, not the ground station

Measured 2026-09-20, vision at 90% of one core, control-loop jitter probe on isolated
CPU 5 (250 Hz, 4000 us period):

| condition | max jitter | missed deadlines |
|---|---|---|
| vision running, no recording | 9.5 us | 0 / 5000 |
| **vision + `ros2 bag record` on the Orin** | **9.9 us** | **0 / 5000** |

**Recording locally costs the control loop nothing measurable** — 0.4 us of difference is
well inside run-to-run variation, and neither run missed a deadline. CPU isolation keeps
the recorder on CPUs 0-3, and NVMe writes are DMA rather than CPU work.

Bag: 13.5 MB over 25 s = **1.9 GB/hour**, 1471 messages. With 192 GB free that is ~100
hours of headroom.

Record on the Orin anyway, even though 4.3 Mbit/s would fit over WiFi:

- **A bag is a record; it must not depend on a radio link.** If WiFi drops mid-flight —
  which is exactly when the data is most interesting — the ground-station bag has a hole
  in it, or ends.
- The image topic is best-effort, so WiFi loss silently drops frames. Making it reliable
  is worse: retransmits over a lossy link cost the *Orin* CPU and add latency.
- The Orin pays the serialisation cost either way, so streaming saves it nothing.

Ground-station recording is a reasonable *addition* for live monitoring, not a
replacement. If bandwidth ever matters, `--ros-image-every 3` cuts the image stream to
10 fps while detections stay at full rate.
