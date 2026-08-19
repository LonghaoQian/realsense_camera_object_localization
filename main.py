"""Live RGB-D object detection and 3D localization on the RealSense D435i.

Press 'q' in the display window to quit.
"""

import cv2

from camera import RealSenseCamera
from detector import ObjectDetector
from localization import localize


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


def main():
    detector = ObjectDetector(model_path="yolov8n.pt", device="cuda", conf_threshold=0.5)

    with RealSenseCamera(width=640, height=480, fps=30) as camera:
        print("Camera started. Press 'q' to quit.")
        try:
            while True:
                color_image, depth_image = camera.get_aligned_frames()
                if color_image is None:
                    continue

                detections = detector.detect(color_image)
                detections3d = localize(camera, detections, depth_image)

                for det in detections3d:
                    X, Y, Z = det.position
                    print(f"{det.label} ({det.confidence:.2f}): X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m")

                vis = draw(color_image.copy(), detections3d)
                cv2.imshow("RealSense D435i - Object Detection & Localization", vis)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
