#!/usr/bin/env python3
"""Publishes system + per-process CPU/memory stats for rosbag recording.

Records evidence that the vision/recording pipeline is not starving the
autopilot: overall and per-core CPU load, memory, and per-process stats for
any process whose command line matches one of the WATCH patterns.

Publishes std_msgs/String containing one JSON object per sample on
/system_monitor/stats (default 2 Hz), so it can be replayed/analyzed offline
with plain Python/pandas, no custom message package needed.

Uses system ROS2 Humble + python3-psutil (sudo apt install python3-psutil),
NOT the project venv.

Usage:
  ./system_monitor_node.py                       # default watch patterns
  ./system_monitor_node.py autopilot vision ...  # override watch patterns
"""

import json
import sys

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_WATCH = ["autopilot", "realsense2_camera", "vrpn", "ros2 bag", "yolo", "main.py"]
PUBLISH_RATE_HZ = 2.0


class SystemMonitor(Node):
    def __init__(self, watch_patterns):
        super().__init__("system_monitor")
        self.watch_patterns = watch_patterns
        self.pub = self.create_publisher(String, "/system_monitor/stats", 10)
        # Prime cpu_percent counters so the first real sample is meaningful.
        psutil.cpu_percent(percpu=True)
        self.procs = {}
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.sample)

    def matched_processes(self):
        for proc in psutil.process_iter(["pid", "cmdline"]):
            cmdline = " ".join(proc.info["cmdline"] or [])
            for pattern in self.watch_patterns:
                if pattern in cmdline and proc.pid != psutil.Process().pid:
                    yield pattern, proc
                    break

    def sample(self):
        vm = psutil.virtual_memory()
        stats = {
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "cpu_percent_per_core": psutil.cpu_percent(percpu=True),
            "loadavg_1min": psutil.getloadavg()[0],
            "mem_used_percent": vm.percent,
            "processes": [],
        }
        seen = set()
        for pattern, proc in self.matched_processes():
            try:
                # cpu_percent needs a persistent Process object between calls;
                # cache by pid so successive samples measure a real interval.
                cached = self.procs.get(proc.pid)
                if cached is None:
                    cached = self.procs[proc.pid] = psutil.Process(proc.pid)
                    cached.cpu_percent()  # prime; reports 0.0 this first call
                seen.add(proc.pid)
                stats["processes"].append({
                    "pattern": pattern,
                    "pid": proc.pid,
                    "name": cached.name(),
                    "cpu_percent": cached.cpu_percent(),
                    "mem_rss_mb": cached.memory_info().rss / 1e6,
                    "num_threads": cached.num_threads(),
                })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self.procs = {pid: p for pid, p in self.procs.items() if pid in seen}
        self.pub.publish(String(data=json.dumps(stats)))


def main():
    watch = sys.argv[1:] or DEFAULT_WATCH
    rclpy.init()
    node = SystemMonitor(watch)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
