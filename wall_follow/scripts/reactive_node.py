#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """
    Implement the Follow-the-Gap algorithm on the car.
    Steps:
      1. Preprocess LiDAR (windowed mean + range cap)
      2. Find closest point, zero out a safety bubble around it
      3. Find the longest free-space gap
      4. Pick the best point (weighted: furthest + centered)
      5. Publish AckermannDriveStamped
    """

    def __init__(self):
        super().__init__('reactive_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.subscription = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)

        # ── Tunable parameters ──────────────────────────────────────────────
        self.declare_parameter('bubble_radius', 0.35)   # safety bubble [m]
        self.declare_parameter('max_range',     3.0)    # cap far readings [m]
        self.declare_parameter('window_size',   5)      # smoothing window [samples]
        self.declare_parameter('weight_far',    0.6)    # weight for furthest point
        self.declare_parameter('weight_center', 0.4)    # weight for gap center

        # Velocity schedule (mirrors your wall-follow approach)
        self.declare_parameter('speed_fast',   1.5)
        self.declare_parameter('speed_medium', 1.0)
        self.declare_parameter('speed_slow',   0.5)

        self.get_logger().info('ReactiveFollowGap node initialized.')

    # ────────────────────────────────────────────────────────────────────────
    # 1. PREPROCESSING
    # ────────────────────────────────────────────────────────────────────────

    def preprocess_lidar(self, ranges):
        """
        1. Replace non-finite values with 0.
        2. Cap readings above max_range.
        3. Apply a rolling mean over `window_size` samples.
        Returns a numpy float array.
        """
        max_range   = self.get_parameter('max_range').get_parameter_value().double_value
        window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        proc = np.array(ranges, dtype=float)

        # Sanitise
        proc = np.where(np.isfinite(proc), proc, 0.0)

        # Cap
        proc = np.clip(proc, 0.0, max_range)

        # Windowed mean via convolution (same length, 'same' pads borders)
        kernel     = np.ones(window_size) / window_size
        proc       = np.convolve(proc, kernel, mode='same')

        return proc

    # ────────────────────────────────────────────────────────────────────────
    # 2. SAFETY BUBBLE
    # ────────────────────────────────────────────────────────────────────────

    def apply_bubble(self, proc_ranges, angle_increment):
        """
        Find the closest point and zero every index within `bubble_radius`
        arc-length of it. Returns modified array + index of closest point.
        """
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value

        closest_idx = int(np.argmin(
            np.where(proc_ranges > 0, proc_ranges, np.inf)
        ))
        closest_dist = proc_ranges[closest_idx]

        if closest_dist == 0.0:
            return proc_ranges, closest_idx

        # Angular half-width of bubble at that distance
        # arc = r * θ  →  θ = bubble_radius / closest_dist (small angle ok here)
        half_angle  = bubble_radius / max(closest_dist, 0.01)
        half_steps  = int(np.ceil(half_angle / angle_increment))

        bubble_start = max(0, closest_idx - half_steps)
        bubble_end   = min(len(proc_ranges) - 1, closest_idx + half_steps)

        proc_ranges[bubble_start:bubble_end + 1] = 0.0

        return proc_ranges, closest_idx

    # ────────────────────────────────────────────────────────────────────────
    # 3. FIND MAX GAP
    # ────────────────────────────────────────────────────────────────────────

    def find_max_gap(self, free_space_ranges):
        """
        Scan through free_space_ranges (zeros = occupied / bubble).
        Return (start_idx, end_idx) of the longest contiguous non-zero run.
        """
        best_start, best_end = 0, 0
        cur_start            = None
        best_len             = 0

        for i, val in enumerate(free_space_ranges):
            if val > 0.0:
                if cur_start is None:
                    cur_start = i
                cur_len = i - cur_start + 1
                if cur_len > best_len:
                    best_len  = cur_len
                    best_start = cur_start
                    best_end   = i
            else:
                cur_start = None

        return best_start, best_end

    # ────────────────────────────────────────────────────────────────────────
    # 4. FIND BEST POINT  (weighted: furthest + centered)
    # ────────────────────────────────────────────────────────────────────────

    def find_best_point(self, start_i, end_i, ranges):
        """
        Weighted combination of:
          • furthest point index  (maximises clearance)
          • center  of gap index  (keeps car away from gap edges)
        weight_far + weight_center should sum to 1.0.
        """
        weight_far    = self.get_parameter('weight_far').get_parameter_value().double_value
        weight_center = self.get_parameter('weight_center').get_parameter_value().double_value

        gap_ranges  = ranges[start_i:end_i + 1]
        furthest_i  = start_i + int(np.argmax(gap_ranges))
        center_i    = (start_i + end_i) // 2

        best_i = int(round(weight_far * furthest_i + weight_center * center_i))
        best_i = np.clip(best_i, start_i, end_i)

        return best_i

    # ────────────────────────────────────────────────────────────────────────
    # 5. LIDAR CALLBACK
    # ────────────────────────────────────────────────────────────────────────

    def lidar_callback(self, data: LaserScan):
        ranges          = data.ranges
        angle_min       = data.angle_min
        angle_increment = data.angle_increment

        # Step 1 – preprocess
        proc_ranges = self.preprocess_lidar(ranges)

        # Step 2 – bubble around closest obstacle
        proc_ranges, _ = self.apply_bubble(proc_ranges, angle_increment)

        # Step 3 – find max gap
        start_i, end_i = self.find_max_gap(proc_ranges)

        # Step 4 – best point in gap
        best_i = self.find_best_point(start_i, end_i, proc_ranges)

        # Convert best index → steering angle
        best_angle = angle_min + best_i * angle_increment   # [rad]

        # Clamp to physical steering limit (same as your wall-follow node)
        MAX_STEER  = 0.35
        steer      = float(np.clip(best_angle, -MAX_STEER, MAX_STEER))

        # Dynamic velocity: ease off the throttle when turning hard
        abs_steer = abs(steer)
        speed_fast   = self.get_parameter('speed_fast').get_parameter_value().double_value
        speed_medium = self.get_parameter('speed_medium').get_parameter_value().double_value
        speed_slow   = self.get_parameter('speed_slow').get_parameter_value().double_value

        if abs_steer < 0.1:
            velocity = speed_fast
        elif abs_steer < 0.2:
            velocity = speed_medium
        else:
            velocity = speed_slow

        # Step 5 – publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed          = velocity
        self.publisher.publish(drive_msg)

        # Uncomment to debug:
        # self.get_logger().info(
        #     f"gap=[{start_i},{end_i}] best={best_i} steer={steer:.3f} v={velocity}"
        # )


def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()