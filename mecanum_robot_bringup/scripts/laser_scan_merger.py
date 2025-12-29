#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf2_ros
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration


class LaserScanMerger(Node):

    def __init__(self):
        super().__init__('laser_scan_merger')

        # ================= PARAMETERS =================
        self.declare_parameter('scan1_topic', '/scan1')
        self.declare_parameter('scan2_topic', '/scan2')
        self.declare_parameter('merged_topic', '/scan_merged')
        self.declare_parameter('target_frame', 'base_link')

        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max',  math.pi)
        self.declare_parameter('angle_increment', 0.0174533)  # 1 deg

        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)

        self.declare_parameter('overlap_method', 'closest')

        self.declare_parameter('scan1_angle_min', -math.pi)
        self.declare_parameter('scan1_angle_max',  math.pi)
        self.declare_parameter('scan2_angle_min', -math.pi)
        self.declare_parameter('scan2_angle_max',  math.pi)

        # ================= GET PARAMS =================
        self.scan1_topic = self.get_parameter('scan1_topic').value
        self.scan2_topic = self.get_parameter('scan2_topic').value
        self.merged_topic = self.get_parameter('merged_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value

        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.overlap_method = self.get_parameter('overlap_method').value

        self.scan1_angle_min = self.get_parameter('scan1_angle_min').value
        self.scan1_angle_max = self.get_parameter('scan1_angle_max').value
        self.scan2_angle_min = self.get_parameter('scan2_angle_min').value
        self.scan2_angle_max = self.get_parameter('scan2_angle_max').value

        self.num_points = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        ) + 1

        # ================= TF =================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ================= STATE =================
        self.scan1 = None
        self.scan2 = None
        self.scan1_stamp = None
        self.scan2_stamp = None

        # ================= PUB / SUB =================
        self.sub1 = self.create_subscription(
            LaserScan, self.scan1_topic, self.cb_scan1, 10)

        self.sub2 = self.create_subscription(
            LaserScan, self.scan2_topic, self.cb_scan2, 10)

        self.pub = self.create_publisher(
            LaserScan, self.merged_topic, 10)

        self.get_logger().info('✅ Optimized LaserScanMerger started')

    # =====================================================
    # CALLBACKS
    # =====================================================
    def cb_scan1(self, msg):
        self.scan1 = msg
        self.scan1_stamp = Time.from_msg(msg.header.stamp)
        self.try_merge()

    def cb_scan2(self, msg):
        self.scan2 = msg
        self.scan2_stamp = Time.from_msg(msg.header.stamp)
        self.try_merge()

    # =====================================================
    # MERGE LOGIC
    # =====================================================
    def try_merge(self):
        if self.scan1 is None or self.scan2 is None:
            return

        # sync check (100 ms)
        dt = abs((self.scan1_stamp - self.scan2_stamp).nanoseconds) / 1e9
        if dt > 0.1:
            return

        try:
            tf1 = self.get_tf(self.scan1)
            tf2 = self.get_tf(self.scan2)
        except Exception:
            return

        ranges = np.full(self.num_points, float('inf'))
        intensities = np.zeros(self.num_points)

        self.process_scan(self.scan1, tf1, ranges, intensities,
                          self.scan1_angle_min, self.scan1_angle_max)
        self.process_scan(self.scan2, tf2, ranges, intensities,
                          self.scan2_angle_min, self.scan2_angle_max)

        self.publish_scan(ranges, intensities)

        # reset để tránh merge scan cũ
        self.scan1 = None
        self.scan2 = None

    # =====================================================
    # TF HANDLING (1 lần / scan)
    # =====================================================
    def get_tf(self, scan):
        tf = self.tf_buffer.lookup_transform(
            self.target_frame,
            scan.header.frame_id,
            Time(),
            Duration(seconds=0.05)
        )

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        return {
            'x': tf.transform.translation.x,
            'y': tf.transform.translation.y,
            'cos': math.cos(yaw),
            'sin': math.sin(yaw)
        }

    # =====================================================
    # PROCESS SCAN (NO TF CALLS HERE)
    # =====================================================
    def process_scan(self, scan, tf, ranges, intensities,
                     angle_min_f, angle_max_f):

        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if angle < angle_min_f or angle > angle_max_f:
                continue

            # sensor frame
            xs = r * math.cos(angle)
            ys = r * math.sin(angle)

            # transform to base_link
            xb = tf['cos'] * xs - tf['sin'] * ys + tf['x']
            yb = tf['sin'] * xs + tf['cos'] * ys + tf['y']

            ang = math.atan2(yb, xb)
            idx = int((ang - self.angle_min) / self.angle_increment)

            if 0 <= idx < self.num_points:
                dist = math.hypot(xb, yb)
                self.update_point(ranges, intensities, idx, dist,
                                  scan.intensities[i] if i < len(scan.intensities) else 0.0)

    # =====================================================
    # OVERLAP HANDLING
    # =====================================================
    def update_point(self, ranges, intensities, idx, r, inten):
        if self.overlap_method == 'closest':
            if r < ranges[idx]:
                ranges[idx] = r
                intensities[idx] = inten

        elif self.overlap_method == 'average':
            if math.isinf(ranges[idx]):
                ranges[idx] = r
                intensities[idx] = inten
            else:
                ranges[idx] = 0.5 * (ranges[idx] + r)
                intensities[idx] = 0.5 * (intensities[idx] + inten)

        elif self.overlap_method == 'newest':
            ranges[idx] = r
            intensities[idx] = inten

    # =====================================================
    # PUBLISH
    # =====================================================
    def publish_scan(self, ranges, intensities):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        msg.ranges = ranges.tolist()
        msg.intensities = intensities.tolist()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
