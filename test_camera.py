"""Quick connectivity check: confirms the D435i streams RGB + depth before running detection."""

import cv2

from camera import RealSenseCamera


def main():
    with RealSenseCamera(width=640, height=480, fps=30) as camera:
        print(f"Depth scale: {camera.depth_scale} meters/unit")
        print(f"Color intrinsics: {camera.intrinsics}")

        color_image, depth_image = camera.get_aligned_frames()
        if color_image is None:
            print("FAILED: no frames received")
            return

        print(f"Color frame shape: {color_image.shape}")
        print(f"Depth frame shape: {depth_image.shape}")

        h, w = depth_image.shape
        center_point = camera.deproject_pixel(w // 2, h // 2, depth_image)
        print(f"3D point at image center: {center_point}")

        cv2.imwrite("test_color.png", color_image)
        print("Saved test_color.png -- inspect it to confirm the feed looks correct.")
        print("OK")


if __name__ == "__main__":
    main()
