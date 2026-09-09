#!/usr/bin/env python3
"""Read-only numerical observer for the actual Nav2 local costmap.

It never publishes.  It reports occupied cells in a caller-supplied region of
the ``/local_costmap/costmap_raw`` OccupancyGrid and cloud-frame health for
the same actual source topic.  The region is deliberately supplied at run
time after its body-frame coordinates have been established from TF; this is
an observer, not an obstacle detector or a replacement costmap.
"""

import json
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CostmapObserver(Node):
    def __init__(self):
        super().__init__("mk2c_costmap_observer")
        # ``costmap_raw`` is nav2_msgs/Costmap. This observer intentionally
        # consumes the equivalent OccupancyGrid output for numerical cells.
        self.declare_parameter("costmap_topic", "/costmap/costmap")
        self.declare_parameter("cloud_topic", "/cloud_registered_body")
        self.declare_parameter("region_x_min_m", 0.0)
        self.declare_parameter("region_x_max_m", 3.0)
        self.declare_parameter("region_y_min_m", -0.5)
        self.declare_parameter("region_y_max_m", 0.5)
        self.declare_parameter("report_period_sec", 1.0)
        self.last_cloud = None
        self.last_grid = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value,
                                 self._cloud, qos)
        self.create_subscription(OccupancyGrid, self.get_parameter("costmap_topic").value,
                                 self._grid, 10)
        self.create_timer(float(self.get_parameter("report_period_sec").value), self._report)

    def _cloud(self, msg):
        self.last_cloud = msg

    def _grid(self, msg):
        self.last_grid = msg

    def _report(self):
        result = {"observer": "P5A_MK2C_READ_ONLY", "cloud_received": self.last_cloud is not None,
                  "costmap_received": self.last_grid is not None}
        if self.last_cloud:
            result["cloud"] = {"frame_id": self.last_cloud.header.frame_id,
                               "width": self.last_cloud.width, "height": self.last_cloud.height,
                               "stamp_sec": self.last_cloud.header.stamp.sec + self.last_cloud.header.stamp.nanosec * 1e-9}
        if self.last_grid:
            g = self.last_grid
            xmin = float(self.get_parameter("region_x_min_m").value)
            xmax = float(self.get_parameter("region_x_max_m").value)
            ymin = float(self.get_parameter("region_y_min_m").value)
            ymax = float(self.get_parameter("region_y_max_m").value)
            cells = []
            for row in range(g.info.height):
                y = g.info.origin.position.y + (row + 0.5) * g.info.resolution
                if not ymin <= y <= ymax:
                    continue
                for col in range(g.info.width):
                    x = g.info.origin.position.x + (col + 0.5) * g.info.resolution
                    if xmin <= x <= xmax:
                        value = g.data[row * g.info.width + col]
                        if value >= 100:
                            cells.append({"x_m": round(x, 3), "y_m": round(y, 3), "cost": value})
            result["costmap"] = {
                "frame_id": g.header.frame_id, "resolution_m": g.info.resolution,
                "region_m": {"x": [xmin, xmax], "y": [ymin, ymax]},
                "occupied_cell_count": len(cells), "occupied_cells": cells[:100],
                "occupied_cells_truncated": len(cells) > 100,
            }
        self.get_logger().info(json.dumps(result, sort_keys=True))


def main():
    rclpy.init()
    node = CostmapObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
