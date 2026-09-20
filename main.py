"""Live RGB-D object detection and 3D localization on the RealSense D435i.

Press 'q' in the display window to quit, or Ctrl-C when running headless.
"""

import argparse

import cv2

from camera import RealSenseCamera
from detector import ObjectDetector
from localization import localize
from stream import MJPEGServer

WINDOW = "RealSense D435i - Object Detection & Localization"


def draw(color_image, detections3d):
    for det in detections3d:
        x1, y1, x2, y2 = det.bbox
        X, Y, Z = det.position
        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{det.label} {det.confidence:.2f} | {X:.2f},{Y:.2f},{Z:.2f}m"
        cv2.putText(
            color_image, text, (x1, max(0, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
        )
    return color_image


def open_window():
    """Try to open the display window; returns False if there's no usable GUI backend.

    Over SSH (or on a headless Jetson) OpenCV's GTK backend fails to initialize,
    so fall back to console output instead of taking the whole loop down.
    """
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        return True
    except cv2.error:
        return False


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="yolov8n.pt", help="Ultralytics model path")
    p.add_argument("--device", default="cuda", help="'cuda' or 'cpu'")
    p.add_argument("--conf", type=float, default=0.5, help="detection confidence threshold")
    p.add_argument(
        "--classes", nargs="+", default=None, metavar="NAME",
        help="only detect these COCO classes, e.g. --classes bottle. "
             "Default: all 80. Filtering happens inside the model, so a "
             "single-class run is cheaper than detecting everything and discarding.",
    )
    p.add_argument(
        "--list-classes", action="store_true",
        help="print the model's class names and exit",
    )
    p.add_argument(
        "--ros", action="store_true",
        help="publish detections (and RGB) as ROS2 topics so the run can be "
             "recorded with `ros2 bag record`. Requires /opt/ros/humble sourced.",
    )
    p.add_argument("--ros-ns", default="/vision", help="ROS namespace for --ros")
    p.add_argument(
        "--ros-no-image", action="store_true",
        help="with --ros, publish detections only (no RGB). Smallest possible bag.",
    )
    p.add_argument(
        "--ros-image-hz", type=float, default=0.0, metavar="HZ",
        help="with --ros, cap RGB publication at HZ (0 = every frame, ~20 Hz). "
             "Time-based, so 15 is achievable even though the loop runs at ~20 Hz. "
             "Detections and pose always stay at full rate.",
    )
    p.add_argument("--ros-jpeg-quality", type=int, default=80, help="JPEG quality for --ros")
    p.add_argument("--headless", action="store_true", help="skip the display window entirely")
    p.add_argument(
        "--stream", nargs="?", const=8080, type=int, default=None, metavar="PORT",
        help="also serve the annotated feed as MJPEG on PORT (default 8080), "
             "viewable in a browser from another machine on the LAN",
    )
    return p.parse_args()


def main():
    args = parse_args()

    if args.list_classes:
        from ultralytics import YOLO
        for idx, name in sorted(YOLO(args.model).names.items()):
            print(f"{idx:3d}  {name}")
        return

    detector = ObjectDetector(
        model_path=args.model, device=args.device, conf_threshold=args.conf,
        classes=args.classes,
    )
    print(f"Detecting classes: {detector.class_names} (conf >= {args.conf})")

    show = not args.headless and open_window()
    if not show and not args.headless:
        print("No usable display (GUI backend unavailable) -- running headless.")

    publisher = None
    if args.ros:
        from ros_publisher import VisionPublisher
        publisher = VisionPublisher(
            namespace=args.ros_ns,
            publish_image=not args.ros_no_image,
            jpeg_quality=args.ros_jpeg_quality,
            image_hz=args.ros_image_hz,
        )

    streamer = None
    if args.stream is not None:
        streamer = MJPEGServer(port=args.stream).start()
        print(f"Streaming annotated feed at {streamer.url()} (open it in a browser)")

    with RealSenseCamera(width=640, height=480, fps=30) as camera:
        print("Camera started. " + ("Press 'q' to quit." if show else "Ctrl-C to quit."))
        try:
            while True:
                color_image, depth_image = camera.get_aligned_frames()
                if color_image is None:
                    continue

                detections = detector.detect(color_image)
                detections3d = localize(camera, detections, depth_image)

                # Publish before drawing: the annotated copy is a display concern and
                # must not sit between detection and the recorded timestamp.
                if publisher is not None:
                    publisher.publish(color_image, detections3d)

                for det in detections3d:
                    X, Y, Z = det.position
                    print(f"{det.label} ({det.confidence:.2f}): X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m")

                # Only pay for the annotated copy when something will look at it.
                if show or (streamer is not None and streamer.has_clients):
                    vis = draw(color_image.copy(), detections3d)
                    if streamer is not None:
                        streamer.publish(vis)
                    if show:
                        cv2.imshow(WINDOW, vis)

                if show and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            if streamer is not None:
                streamer.stop()
            if publisher is not None:
                publisher.shutdown()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
