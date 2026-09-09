#!/usr/bin/env python3
"""Read-only MK2C1R cloud/height/costmap reconciliation observer.

This observer never publishes and is not an obstacle classifier.  For known
operator-placed box clearances it transforms the real FAST-LIO cloud into the
actual costmap frame, applies the configured VoxelLayer Z band there, and
counts both accepted cloud points and real Nav2 occupied cells in the same
physical windows.  Clear and box captures can therefore be compared without
reimplementing costmap logic.
"""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z + x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def transform_points(points, transform):
    t = transform.transform.translation
    return points @ rotation_matrix(transform.transform.rotation).T + np.array([t.x, t.y, t.z])


def summarize(values):
    if not values:
        return {"samples": 0, "min": None, "median": None, "max": None, "mean": None}
    a = np.asarray(values, dtype=float)
    return {"samples": len(values), "min": float(a.min()), "median": float(np.median(a)),
            "max": float(a.max()), "mean": float(a.mean())}


class ReconciliationObserver(Node):
    def __init__(self):
        super().__init__("mk2c1r_reconciliation_observer")
        self.declare_parameter("cloud_topic", "/cloud_registered_body")
        self.declare_parameter("costmap_topic", "/costmap/costmap")
        self.declare_parameter("costmap_frame", "odom_chassis")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("clearances_m", [0.595, 0.295])
        self.declare_parameter("front_edge_x_m", 0.755)
        self.declare_parameter("box_length_x_m", 0.210)
        self.declare_parameter("box_width_y_m", 0.310)
        self.declare_parameter("box_height_z_m", 0.430)
        self.declare_parameter("xy_padding_m", 0.10)
        self.declare_parameter("z_padding_m", 0.03)
        self.declare_parameter("min_obstacle_height_m", 0.10)
        self.declare_parameter("max_obstacle_height_m", 1.60)
        self.declare_parameter("required_cloud_samples", 100)
        self.declare_parameter("required_costmap_samples", 40)
        self.declare_parameter("output_path", "/tmp/mk2c1r_reconciliation.json")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.cloud_counts = self._empty_counts()
        self.costmap_counts = self._empty_counts(costmap=True)
        self.latest_windows = {}
        self.cloud_frames = set()
        self.costmap_frames = set()
        self.tf_failures = []
        self.forward_profile = {
            f"{start / 10:.1f}-{(start + 1) / 10:.1f}": []
            for start in range(0, 30)
        }
        self.done = False
        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value,
                                 self._cloud, 10)
        self.create_subscription(OccupancyGrid, self.get_parameter("costmap_topic").value,
                                 self._costmap, 10)
        self.create_timer(0.25, self._maybe_finish)

    def _labels(self):
        return [(f"clearance_{float(v):.3f}m", float(v))
                for v in self.get_parameter("clearances_m").value]

    def _empty_counts(self, costmap=False):
        leaf = (lambda: {"occupied_cells": []}) if costmap else (
            lambda: {"points_in_box_xyz_before_height_filter": [],
                     "points_in_same_xy_all_z_before_height_filter": [],
                     "points_in_box_xyz_accepted_after_height_filter": [],
                     "points_in_same_xy_accepted_after_height_filter": []})
        return {name: leaf() for name, _ in self._labels()}

    def _lookup(self, target, source, stamp):
        return self.tf.lookup_transform(target, source, stamp, Duration(seconds=0.3))

    def _window(self, base_tf, clearance):
        front = float(self.get_parameter("front_edge_x_m").value)
        length = float(self.get_parameter("box_length_x_m").value)
        half_width = float(self.get_parameter("box_width_y_m").value) / 2
        height = float(self.get_parameter("box_height_z_m").value)
        corners = np.array([[x, y, z] for x in (front + clearance, front + clearance + length)
                            for y in (-half_width, half_width) for z in (0.0, height)])
        q = transform_points(corners, base_tf)
        xy_pad = float(self.get_parameter("xy_padding_m").value)
        z_pad = float(self.get_parameter("z_padding_m").value)
        lo, hi = q.min(axis=0), q.max(axis=0)
        lo -= np.array([xy_pad, xy_pad, z_pad]); hi += np.array([xy_pad, xy_pad, z_pad])
        return lo, hi

    @staticmethod
    def _xyz(msg):
        raw = np.asarray(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if raw.dtype.fields:
            return np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(float, copy=False)
        return raw.reshape((-1, 3)).astype(float, copy=False)

    def _cloud(self, msg):
        if self.done:
            return
        target = self.get_parameter("costmap_frame").value
        try:
            stamp = Time.from_msg(msg.header.stamp)
            cloud_tf = self._lookup(target, msg.header.frame_id, stamp)
            base_tf = self._lookup(target, self.get_parameter("base_frame").value, stamp)
            cloud_to_base = self._lookup(self.get_parameter("base_frame").value,
                                         msg.header.frame_id, stamp)
        except Exception as exc:
            self.tf_failures.append(f"cloud: {exc}")
            return
        points_source = self._xyz(msg)
        points_costmap = transform_points(points_source, cloud_tf)
        points_base = transform_points(points_source, cloud_to_base)
        profile_points = points_base[(points_base[:, 2] >= 0.05) &
                                     (points_base[:, 2] <= 0.60) &
                                     (np.abs(points_base[:, 1]) <= 0.80)]
        for start in range(0, 30):
            lo_x, hi_x = start / 10, (start + 1) / 10
            self.forward_profile[f"{lo_x:.1f}-{hi_x:.1f}"].append(
                int(np.count_nonzero((profile_points[:, 0] >= lo_x) &
                                     (profile_points[:, 0] < hi_x))))
        zmin = float(self.get_parameter("min_obstacle_height_m").value)
        zmax = float(self.get_parameter("max_obstacle_height_m").value)
        accepted = points_costmap[(points_costmap[:, 2] >= zmin) & (points_costmap[:, 2] <= zmax)]
        self.cloud_frames.add(msg.header.frame_id)
        for name, clearance in self._labels():
            lo, hi = self._window(base_tf, clearance)
            self.latest_windows[name] = {"min": lo.tolist(), "max": hi.tolist()}
            box_before = points_costmap[np.all((points_costmap >= lo) & (points_costmap <= hi), axis=1)]
            xy_before = points_costmap[np.all((points_costmap[:, :2] >= lo[:2]) &
                                              (points_costmap[:, :2] <= hi[:2]), axis=1)]
            box_after = accepted[np.all((accepted >= lo) & (accepted <= hi), axis=1)]
            xy_after = accepted[np.all((accepted[:, :2] >= lo[:2]) &
                                       (accepted[:, :2] <= hi[:2]), axis=1)]
            groups = self.cloud_counts[name]
            groups["points_in_box_xyz_before_height_filter"].append(len(box_before))
            groups["points_in_same_xy_all_z_before_height_filter"].append(len(xy_before))
            groups["points_in_box_xyz_accepted_after_height_filter"].append(len(box_after))
            groups["points_in_same_xy_accepted_after_height_filter"].append(len(xy_after))

    def _costmap(self, msg):
        if self.done:
            return
        target = msg.header.frame_id
        try:
            stamp = Time.from_msg(msg.header.stamp)
            base_tf = self._lookup(target, self.get_parameter("base_frame").value, stamp)
        except Exception as exc:
            self.tf_failures.append(f"costmap: {exc}")
            return
        self.costmap_frames.add(target)
        data = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        for name, clearance in self._labels():
            lo, hi = self._window(base_tf, clearance)
            c0 = max(0, int(math.floor((lo[0] - msg.info.origin.position.x) / msg.info.resolution)))
            c1 = min(msg.info.width, int(math.ceil((hi[0] - msg.info.origin.position.x) / msg.info.resolution)))
            r0 = max(0, int(math.floor((lo[1] - msg.info.origin.position.y) / msg.info.resolution)))
            r1 = min(msg.info.height, int(math.ceil((hi[1] - msg.info.origin.position.y) / msg.info.resolution)))
            count = int(np.count_nonzero(data[r0:r1, c0:c1] >= 100)) if c1 > c0 and r1 > r0 else 0
            self.costmap_counts[name]["occupied_cells"].append(count)

    def _maybe_finish(self):
        cloud_n = min(len(v["points_in_box_xyz_accepted_after_height_filter"])
                      for v in self.cloud_counts.values())
        grid_n = min(len(v["occupied_cells"]) for v in self.costmap_counts.values())
        if cloud_n < int(self.get_parameter("required_cloud_samples").value) or \
                grid_n < int(self.get_parameter("required_costmap_samples").value):
            return
        self.done = True
        result = {
            "observer": "P5A_MK2C1R_READ_ONLY_FRAME_RECONCILIATION",
            "classification_authority": "actual Nav2 VoxelLayer/costmap delta, not this observer",
            "costmap_frame_configured": self.get_parameter("costmap_frame").value,
            "source_cloud_frames": sorted(self.cloud_frames),
            "observed_costmap_frames": sorted(self.costmap_frames),
            "height_filter_applied_after_transform_to_costmap_frame_m": [
                float(self.get_parameter("min_obstacle_height_m").value),
                float(self.get_parameter("max_obstacle_height_m").value)],
            "physical_windows_in_costmap_frame_latest": self.latest_windows,
            "cloud": {name: {key: summarize(vals) for key, vals in groups.items()}
                      for name, groups in self.cloud_counts.items()},
            "base_frame_forward_profile": {
                "method": "base_footprint Z 0.05..0.60 m, abs(Y)<=0.80 m, X bins 0.10 m",
                "bins": {name: summarize(values)
                         for name, values in self.forward_profile.items()},
            },
            "costmap": {name: {key: summarize(vals) for key, vals in groups.items()}
                        for name, groups in self.costmap_counts.items()},
            "tf_failure_count": len(self.tf_failures),
            "tf_failure_examples": self.tf_failures[:10],
        }
        path = Path(self.get_parameter("output_path").value)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(json.dumps(result, sort_keys=True))
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ReconciliationObserver()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
