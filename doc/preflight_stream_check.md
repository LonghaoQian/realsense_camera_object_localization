# Pre-flight: check the camera feed in a browser

A five-minute check to run **before** a data-collection session, from a laptop
over SSH: confirm the D435i is seen, streaming, correctly exposed, and
producing sane depth — while you can still fix it, rather than discovering a
bad take after the fact.

Ends by stopping the stream, because the recording pipeline needs the camera
free. Read that last step before you start.

Values below are for the Orin Nano `yahboom` at `192.168.50.107` (camera
serial `244622072041`). Setup for the box itself is in
[`jetson_orin_setup.md`](jetson_orin_setup.md) — this assumes that is done.

---

## 1. Connect and confirm the camera is there

```bash
ssh jetson@192.168.50.107
cd ~/source/realsense_camera_object_localization
source .venv/bin/activate

python -c "
import pyrealsense2 as rs
for d in rs.context().query_devices():
    print(d.get_info(rs.camera_info.name),
          '| serial', d.get_info(rs.camera_info.serial_number),
          '| USB', d.get_info(rs.camera_info.usb_type_descriptor))
"
```

Expected:

```
RealSense D435I | serial 244622072041 | USB 3.2
```

Two things to actually look at, not just glance past:

- **`USB 3.2`.** If it says `2.1`, the camera is on a USB 2 link and *cannot*
  sustain 640x480 color + depth at 30 fps — you will silently drop frames all
  session. Move it to one of the stacked blue USB 3 ports and re-check.
- **The serial.** Confirms it is the camera you calibrated, if more than one
  is ever around.

No output at all means the camera is not enumerating: reseat the USB cable
(the D435i is sensitive to marginal cables — use the one it shipped with) and
re-run. This step is safe to run at any time; it does not open the stream.

## 2. Start the stream

```bash
python main.py --stream
```

It prints two lines and then starts reporting detections:

```
No usable display (GUI backend unavailable) -- running headless.
Streaming annotated feed at http://192.168.50.107:8080/ (open it in a browser)
Camera started. Ctrl-C to quit.
person (0.56): X=-0.25m Y=-0.90m Z=2.32m
```

The "No usable display" line is **normal and expected** over SSH — JetPack's
OpenCV cannot open a window on a forwarded X display, so it falls back to
console output plus the stream. It is not an error.

First start takes ~10-15 s while YOLO loads onto the GPU.

**Trust the printed URL over the one in this doc.** The Orin's address is a
DHCP lease, so it changes if the lab moves to a different router — see
"Finding the host's IP address" in the [README](../README.md) for how to
recover it (and for the WiFi-only caveat, which is what actually bites: if the
Orin cannot join the network, there is no address to find).

> Run this in the **foreground** for a pre-flight check, so `Ctrl-C` ends it
> and you cannot forget it is running. A detached tmux session left holding
> the camera is the single most likely way to break the recording in step 5.
> Use `scripts/run_detection_tmux.sh --stream` only for long unattended runs.

## 3. Open it in the browser

On the laptop, go to:

```
http://192.168.50.107:8080/
```

Nothing to install — it is an MJPEG stream in a plain `<img>` tag, which every
browser handles natively. Multiple people can open it at once.

## 4. What to check on the image

Walk an object (a person works) through the frame at a known distance and
confirm all four:

| Check | Good | Bad — and what it means |
| --- | --- | --- |
| **Live** | Motion appears with no more than ~0.2 s lag | Frozen image = detection loop stalled; check the console |
| **Exposure** | Scene clearly visible | Very dark or blown out — the lab lighting will wreck detection accuracy; fix the lighting before collecting |
| **Boxes** | Green boxes track objects, labels sensible | No boxes on an obvious person = model or threshold problem; try `--conf 0.3` |
| **Depth** | Console `Z=` roughly matches a tape measure | `Z` wildly off, or no console lines while boxes are drawn, means depth is invalid in that region |

The console `X, Y, Z` are metres in the **color optical frame** — X right, Y
down, Z forward out of the lens. `Z` is the one to sanity-check against a tape
measure; a detection at 2 m should read `Z≈2.0`.

Detections whose depth patch is entirely invalid are **skipped silently** —
so a box drawn on screen with no matching console line is itself the signal
that the surface returns no depth (reflective, transparent, or out of range).
Worth knowing before you build a session around such an object.

Reference throughput on this box, for comparison if it feels slow: ~30 fps to
the browser, ~24 fps end-to-end for the detection loop.

## 5. Stop the stream before recording — required

```
Ctrl-C
```

**Only one process can hold the D435i at a time.** The ROS2 recording path
(`ros-humble-realsense2-camera`, see
[`recording_with_optitrack.md`](recording_with_optitrack.md)) drives the same
physical camera through a separate stack, and it will fail to open the device
if `main.py` still has it:

```
xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy
```

If you see that when launching the recording, something still holds the
camera. Find and stop it:

```bash
scripts/tmux_status.sh          # a forgotten detached run?
pgrep -af "main.py"             # or a stray foreground one
tmux kill-session -t rs_detect  # stop the detached run
```

Confirm it is released by re-running step 1 — enumeration works either way, so
the real proof is that the recording launches cleanly.

---

## Troubleshooting

**Browser shows "can't connect" / times out**

Check the server is actually listening, from the Orin:

```bash
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8080/    # expect 200
ss -tln | grep 8080                                                # expect 0.0.0.0:8080
```

`200` from the Orin but nothing from the laptop means a network problem, not a
camera one: confirm both are on the same subnet (`hostname -I` on the Orin vs
your laptop's address — both should be `192.168.50.x`) and that no firewall
sits between them.

**`OSError: [Errno 98] Address already in use`**

Port 8080 is taken, almost always by an earlier copy of this script:

```bash
ss -tlnp | grep 8080     # shows the holding pid
```

Stop that one, or just use another port — `python main.py --stream 9000` — and
open the new port in the browser.

**`Device or resource busy` on startup**

Something else already has the camera. See step 5.

**`RuntimeError: Frame didn't arrive within 5000`**

The camera enumerates fine (`lsusb` shows it, `USB 3.2`, the pipeline reports
it started) but never delivers a frame. The device is in a stuck streaming
state, which happens after an unclean stop — a SIGKILLed process, a
`tmux kill-server`, a crash mid-stream. It does **not** clear by re-running.

```bash
scripts/reset_camera.sh
```

That issues a firmware reset over USB (equivalent to replugging, but works
over SSH) and then verifies frames actually flow, so a pass means the next
`main.py` will work. Takes ~10 s. It refuses to run while `main.py` still
holds the camera, so stop that first.

If it reports `Still broken: only N/10 frames arrived`, the reset was not
enough: replug the cable, or move to the other USB 3 port. A marginal cable is
the usual culprit — use the one the camera shipped with.

**Page loads but the image never appears**

The page renders before the first frame arrives. Give it ~15 s on first start
while the model loads. If it stays blank, check the console for a traceback.

**Harmless noise in the console**

`libEGL warning: DRI3/DRI2: failed to authenticate` and
`nvbufsurftransform: Could not get EGL display connection` appear on every
headless run and mean nothing here.

---

## Quick reference

```bash
ssh jetson@192.168.50.107
cd ~/source/realsense_camera_object_localization && source .venv/bin/activate
python main.py --stream                    # then open http://192.168.50.107:8080/
                                           # Ctrl-C before recording
```

Security note: the stream binds `0.0.0.0` with no authentication, so anyone on
the LAN who knows the port can watch the camera. Fine in the lab; remember it
elsewhere.
