#!/usr/bin/env python3
"""Read-only reproducible practical-floor-return analysis for MK2C.

Cloud points are transformed from their header frame into base_footprint using
the live TF tree.  A robust horizontal plane is fit from a defined central
floor search corridor, then 0.10 m x-bins are marked persistent only when
they have both aggregate density and multi-frame support.  This reports room/
project-cloud evidence, never a universal sensor FOV or blind-zone limit.
"""

import csv
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


class FloorObserver(Node):
    def __init__(self):
        super().__init__("mk2c_floor_return_observer")
        for name, default in [
            ("cloud_topic", "/cloud_registered_body"), ("base_frame", "base_footprint"),
            ("frame_count", 20), ("center_half_width_m", 0.30),
            ("x_min_m", 0.40), ("x_max_m", 4.00), ("x_bin_m", 0.10),
            ("plane_tolerance_m", 0.05), ("min_points_per_bin", 30),
            ("min_persistent_fraction", 0.50), ("max_floor_slope", 0.20),
            ("output_prefix", "/tmp/mk2c_floor_return"),
        ]:
            self.declare_parameter(name, default)
        self.frames = []
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value,
                                 self._cloud, 10)

    def _cloud(self, msg):
        if len(self.frames) >= int(self.get_parameter("frame_count").value):
            return
        try:
            tf = self.tf.lookup_transform(self.get_parameter("base_frame").value,
                                          msg.header.frame_id, Time(), Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warning(f"waiting for base transform: {exc}")
            return
        raw = np.asarray(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))
        if raw.dtype.fields:
            p = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(float, copy=False)
        else:
            p = raw.reshape((-1, 3)).astype(float, copy=False)
        if not len(p):
            return
        t = tf.transform.translation
        p = p @ rotation(tf.transform.rotation).T + np.array([t.x, t.y, t.z])
        self.frames.append(p)
        if len(self.frames) == int(self.get_parameter("frame_count").value):
            self._analyze()
            rclpy.shutdown()

    def _analyze(self):
        x0, x1 = float(self.get_parameter("x_min_m").value), float(self.get_parameter("x_max_m").value)
        half = float(self.get_parameter("center_half_width_m").value)
        tol = float(self.get_parameter("plane_tolerance_m").value)
        allp = np.vstack(self.frames)
        search = allp[(allp[:, 0] >= x0) & (allp[:, 0] <= x1) & (np.abs(allp[:, 1]) <= half)]
        # A low 20th-percentile horizontal candidate is only a seed; iterative
        # least squares with a narrow residual band is the actual plane fit.
        seed = np.quantile(search[:, 2], 0.20)
        band = search[np.abs(search[:, 2] - seed) <= 0.15]
        A = np.column_stack([band[:, 0], band[:, 1], np.ones(len(band))])
        co, *_ = np.linalg.lstsq(A, band[:, 2], rcond=None)
        for _ in range(2):
            residual = search[:, 2] - (co[0] * search[:, 0] + co[1] * search[:, 1] + co[2])
            inliers = search[np.abs(residual) <= tol]
            A = np.column_stack([inliers[:, 0], inliers[:, 1], np.ones(len(inliers))])
            co, *_ = np.linalg.lstsq(A, inliers[:, 2], rcond=None)
        slope = float(math.hypot(co[0], co[1]))
        if slope > float(self.get_parameter("max_floor_slope").value):
            result = {"classification": "PROJECT_CLOUD_PRACTICAL_NEAREST_FLOOR_RETURN_INVALID",
                      "cloud_frame_transformed_to": self.get_parameter("base_frame").value,
                      "plane_z_equals_ax_by_plus_c": {"a": float(co[0]), "b": float(co[1]), "c": float(co[2])},
                      "slope_m_per_m": slope,
                      "max_accepted_slope_m_per_m": float(self.get_parameter("max_floor_slope").value),
                      "nearest_persistent_bin": None,
                      "uncertainty": "No sufficiently horizontal plane was established; no floor-return boundary is reported."}
            prefix = Path(self.get_parameter("output_prefix").value)
            prefix.parent.mkdir(parents=True, exist_ok=True)
            Path(str(prefix) + ".json").write_text(json.dumps(result, indent=2) + "\n")
            self.get_logger().warning(json.dumps(result, sort_keys=True))
            return
        size = float(self.get_parameter("x_bin_m").value)
        nbin = int(math.ceil((x1 - x0) / size))
        rows = []
        needed_frames = math.ceil(len(self.frames) * float(self.get_parameter("min_persistent_fraction").value))
        for i in range(nbin):
            lo, hi = x0 + i * size, min(x1, x0 + (i + 1) * size)
            total, supported = 0, 0
            for frame in self.frames:
                q = frame[(frame[:, 0] >= lo) & (frame[:, 0] < hi) & (np.abs(frame[:, 1]) <= half)]
                residual = q[:, 2] - (co[0] * q[:, 0] + co[1] * q[:, 1] + co[2])
                c = int(np.count_nonzero(np.abs(residual) <= tol))
                total += c
                supported += c > 0
            persistent = total >= int(self.get_parameter("min_points_per_bin").value) and supported >= needed_frames
            rows.append({"x_bin_min_m": lo, "x_bin_max_m": hi, "floor_points": total,
                         "frames_with_floor_points": supported, "persistent": persistent})
        nearest = next((r for r in rows if r["persistent"]), None)
        result = {"classification": "PROJECT_CLOUD_PRACTICAL_NEAREST_FLOOR_RETURN",
                  "cloud_frame_transformed_to": self.get_parameter("base_frame").value,
                  "plane_z_equals_ax_by_plus_c": {"a": float(co[0]), "b": float(co[1]), "c": float(co[2])},
                  "method": {"center_corridor_abs_y_m": half, "x_range_m": [x0, x1],
                             "x_bin_m": size, "plane_tolerance_m": tol,
                             "minimum_bin_points": int(self.get_parameter("min_points_per_bin").value),
                             "minimum_frame_support": needed_frames, "sampled_frames": len(self.frames)},
                  "nearest_persistent_bin": nearest,
                  "uncertainty": "Room/project-cloud evidence only; not a universal MID-360 FOV or blind-zone result."}
        prefix = Path(self.get_parameter("output_prefix").value)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        with open(str(prefix) + ".csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
        Path(str(prefix) + ".json").write_text(json.dumps(result, indent=2) + "\n")
        self.get_logger().info(json.dumps(result, sort_keys=True))


def main():
    rclpy.init()
    node = FloorObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
