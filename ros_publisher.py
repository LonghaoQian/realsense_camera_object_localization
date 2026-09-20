"""Optional ROS 2 publishing for the detection loop, so runs can be recorded with `ros2 bag`.

This project is deliberately NOT a ROS package -- no package.xml, no colcon build. rclpy
works fine from a plain script once /opt/ros/humble/setup.bash is sourced, and keeping it
standalone means `python main.py` still works with no ROS at all. Import of this module is
therefore lazy: nothing here is touched unless `--ros` is passed.

Why standard messages only (sensor_msgs / geometry_msgs / std_msgs): defining a custom
Detection3D message would require turning this into a package with a CMake/ament build.
`vision_msgs` (which has exactly the right Detection3DArray type) is not installed on this
box either. Standard types cost nothing and keep the project buildless.

WHY COMPRESSED IMAGES, NOT RAW -- this is the decision that makes recording viable:

    raw sensor_msgs/Image  640x480x3 @ 30 fps = 27.6 MB/s
    JPEG CompressedImage   same stream        = ~1-6 MB/s depending on scene

Raw is a non-starter: it saturates WiFi outright (so ground-station recording is
impossible), and on the Orin it is ~100 GB/hour of NVMe writes plus the memory-bandwidth
cost of moving it, which is the one resource CPU isolation does NOT partition. Compressed
makes both recording locations feasible. Record raw only if a downstream step genuinely
needs lossless pixels, and then only locally.
"""

import json
import time

import cv2
import numpy as np


class VisionPublisher:
    """Publishes detections (and optionally the RGB frame) as ROS 2 topics.

    Publish-only: no executor is spun, because this process is driven by the camera
    loop, not by ROS callbacks. `rclpy.init()` plus `publisher.publish()` is all that
    is needed for data to reach the graph and therefore `ros2 bag record`.
    """

    def __init__(self, namespace="/vision", publish_image=True, jpeg_quality=80,
                 image_every=1, frame_id="camera_color_optical_frame"):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._CompressedImage = CompressedImage
        self._PoseStamped = PoseStamped
        self._String = String

        if not rclpy.ok():
            rclpy.init()
        ns = namespace.strip("/")
        self.node = Node("vision_localization", namespace=ns if ns else None)

        self.frame_id = frame_id
        self.publish_image = publish_image
        self.jpeg_quality = int(jpeg_quality)
        self.image_every = max(1, int(image_every))
        self._frame_counter = 0

        # Detections and pose go out RELIABLE (default QoS): they are small, and a
        # dropped detection is a hole in the record that cannot be reconstructed.
        # The image stream uses sensor-data QoS (best effort) -- the standard choice
        # for camera topics, and a dropped frame is recoverable context, not data.
        self.pub_detections = self.node.create_publisher(String, "detections", 10)
        self.pub_target = self.node.create_publisher(PoseStamped, "target/pose", 10)
        self.pub_image = (
            self.node.create_publisher(CompressedImage, "color/compressed",
                                       qos_profile_sensor_data)
            if publish_image else None
        )

        self.node.get_logger().info(
            f"publishing on {self.node.get_namespace().rstrip('/')}/"
            f"{{detections, target/pose"
            + (", color/compressed" if publish_image else "")
            + "}"
        )

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def publish(self, color_image, detections3d):
        """Call once per processed frame. `detections3d` is a list of Detection3D."""
        stamp = self._stamp()

        # Full record as JSON. Self-describing, trivially parseable offline, and it
        # survives adding fields later without a message-definition change.
        payload = {
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "frame_id": self.frame_id,
            "count": len(detections3d),
            "detections": [
                {
                    "label": d.label,
                    "confidence": round(float(d.confidence), 4),
                    "bbox": [int(v) for v in d.bbox],
                    "position": [round(float(v), 4) for v in d.position],
                }
                for d in detections3d
            ],
        }
        msg = self._String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub_detections.publish(msg)

        # Highest-confidence detection as a pose, so a downstream node can consume the
        # target directly without parsing JSON. Nothing is published when there is no
        # detection: an empty PoseStamped would be indistinguishable from "target at the
        # camera origin", which is a genuinely dangerous ambiguity for a controller.
        if detections3d:
            best = max(detections3d, key=lambda d: d.confidence)
            pose = self._PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(best.position[0])
            pose.pose.position.y = float(best.position[1])
            pose.pose.position.z = float(best.position[2])
            pose.pose.orientation.w = 1.0  # position-only; no orientation is estimated
            self.pub_target.publish(pose)

        if self.pub_image is not None:
            self._frame_counter += 1
            if self._frame_counter % self.image_every == 0:
                ok, buf = cv2.imencode(
                    ".jpg", color_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                )
                if ok:
                    img = self._CompressedImage()
                    img.header.stamp = stamp
                    img.header.frame_id = self.frame_id
                    img.format = "jpeg"
                    img.data = buf.tobytes()
                    self.pub_image.publish(img)

    def shutdown(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
