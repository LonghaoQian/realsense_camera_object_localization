"""Optional ROS 2 publishing for the detection loop, so runs can be recorded with `ros2 bag`.

This project is deliberately NOT a ROS package -- no package.xml, no colcon build. rclpy
works fine from a plain script once /opt/ros/humble/setup.bash is sourced, and keeping it
standalone means `python main.py` still works with no ROS at all. Import of this module is
therefore lazy: nothing here is touched unless `--ros` is passed.

Why standard messages only (sensor_msgs / geometry_msgs / std_msgs): defining a custom
Detection3D message would require turning this into a package with a CMake/ament build.
`vision_msgs` (which has exactly the right Detection3DArray type) is not installed on this
box either. Standard types cost nothing and keep the project buildless.

MEASURED COST OF JPEG ENCODING (2026-09-20, real frames, 640x480, this board):

    quality   ms/frame   kB/frame   % of one core @ 20 Hz
        90       3.22       37.7        6.4
        80       2.64       23.8        5.3
        60       2.51       15.5        5.0
        50       2.48       13.6        5.0

That table is the ENCODER in isolation, and in context it is misleading -- see
doc/jetson_orin_setup.md, "What publishing actually costs". Measured in the running loop,
publishing adds no CPU at all: the process is saturated at ~95% of one core whether or not
--ros is passed. The cost appears as LOOP RATE, and it tracks message BYTES rather than
encode work: q80 vs q60 differs by 0.13 ms/frame of encode (~0.1%) but 50% in message size,
and costs 5% of loop rate. The expensive part is CDR serialization, the tobytes() copy and
the DDS write -- not the encoder.

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
                 image_hz=0.0, frame_id="camera_color_optical_frame"):
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
        # Rate limit by TIME, not by frame count. The loop runs at whatever rate
        # detection allows (~20 Hz on this board, not the camera's 30), so counting
        # frames can only give 20/10/6.7 Hz -- you cannot ask for 15. A wall-clock
        # limiter gives the rate actually requested regardless of loop speed.
        self.image_period = (1.0 / image_hz) if image_hz and image_hz > 0 else 0.0
        self._next_image_time = 0.0

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
            now = time.monotonic()
            if self.image_period == 0.0 or now >= self._next_image_time:
                # Advance the schedule by exactly one period so the AVERAGE rate is the
                # one asked for. Scheduling from `now` instead looks simpler but silently
                # under-delivers: a publish can only happen on a loop tick, so `now +
                # period` rounds up to the next tick every time. Measured with a ~25 Hz
                # loop, asking for 15 Hz that way produced 12.5 Hz.
                #
                # The clamp stops it becoming a catch-up burst: if we have fallen more
                # than a period behind (loop stalled, or the requested rate exceeds the
                # loop rate), restart the schedule from now rather than firing repeatedly
                # to "make up" frames that are no longer current.
                self._next_image_time += self.image_period
                if self._next_image_time < now:
                    self._next_image_time = now + self.image_period
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
