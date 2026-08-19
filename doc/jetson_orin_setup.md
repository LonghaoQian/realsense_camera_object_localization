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
