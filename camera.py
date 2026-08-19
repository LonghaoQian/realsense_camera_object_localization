"""RealSense D435i camera wrapper: aligned RGB-D streaming and intrinsics access."""

import numpy as np
import pyrealsense2 as rs


class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Depth-to-color alignment so every depth pixel lines up with its RGB pixel.
        self.align = rs.align(rs.stream.color)

        self.profile = None
        self.depth_scale = None
        self.intrinsics = None

    def start(self):
        self.profile = self.pipeline.start(self.config)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()  # meters per depth unit

        color_stream = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        return self

    def stop(self):
        self.pipeline.stop()

    def get_aligned_frames(self):
        """Returns (color_image[H,W,3] BGR uint8, depth_image[H,W] uint16 raw units) or (None, None)."""
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image

    def deproject_pixel(self, x, y, depth_image):
        """Convert a pixel (x, y) + its depth reading into a 3D point in the camera frame (meters)."""
        depth_raw = depth_image[int(y), int(x)]
        if depth_raw == 0:
            return None  # no valid depth at this pixel
        depth_m = float(depth_raw) * self.depth_scale
        point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(x), float(y)], depth_m)
        return np.array(point)  # [X, Y, Z] meters, camera-optical frame

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
