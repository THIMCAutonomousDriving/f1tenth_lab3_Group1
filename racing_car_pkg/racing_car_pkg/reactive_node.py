#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive_gf', 10)

        self.declare_parameter('bubble_radius', 0.19) #0.2
        self.declare_parameter('max_range',     5.11) 
        self.declare_parameter('window_size',   5)
        self.declare_parameter('weight_far',    0.4)
        self.declare_parameter('weight_center', 0.6)
        #setup 1: 1.5 0.7 0.4
        self.declare_parameter('speed_fast',    1.4) #
        self.declare_parameter('speed_medium',  0.6) #
        self.declare_parameter('speed_slow',    0.3) #
        self.declare_parameter('min_gap_size',  30)

        self.get_logger().info('ReactiveFollowGap node initialized.')

    def preprocess_lidar(self, ranges):
        max_range   = self.get_parameter('max_range').get_parameter_value().double_value
        window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        proc = np.array(ranges, dtype=float)
        proc = np.where(np.isfinite(proc), proc, max_range)
        proc = np.clip(proc, 0.0, max_range)

        kernel = np.ones(window_size) / window_size
        pad_width = window_size // 2
        proc_padded = np.pad(proc, (pad_width, pad_width), mode='edge')
        proc = np.convolve(proc_padded, kernel, mode='valid')

        return proc

    def disparity_extender(self, ranges, angle_increment):
        extended      = ranges.copy()
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value

        for i in range(1, len(ranges)):
            # THE FIX: Ignore artificial cliffs created by the 90-degree 0.0 crop
            if ranges[i] == 0.0 or ranges[i - 1] == 0.0:
                continue

            if abs(ranges[i] - ranges[i - 1]) > 0.3:
                if ranges[i] < ranges[i - 1]:
                    close_dist = max(ranges[i], 0.01)
                    extend_size = int(np.ceil(np.arctan((bubble_radius / 2.0) / close_dist) / angle_increment))
                    start = max(0, i - extend_size)
                    extended[start:i] = np.minimum(extended[start:i], ranges[i])
                else:
                    close_dist = max(ranges[i - 1], 0.01)
                    extend_size = int(np.ceil(np.arctan((bubble_radius / 2.0) / close_dist) / angle_increment))
                    end = min(len(ranges), i + extend_size)
                    extended[i:end] = np.minimum(extended[i:end], ranges[i - 1])

        return extended

    def apply_bubble(self, proc_ranges, angle_increment):
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value

        closest_idx = int(np.argmin(
            np.where(proc_ranges > 0, proc_ranges, np.inf)
        ))
        closest_dist = proc_ranges[closest_idx]

        if closest_dist == 0.0:
            return proc_ranges, closest_idx

        half_angle = bubble_radius / max(closest_dist, 0.01)
        half_steps = int(np.ceil(half_angle / angle_increment))

        bubble_start = max(0, closest_idx - half_steps)
        bubble_end   = min(len(proc_ranges) - 1, closest_idx + half_steps)

        proc_ranges[bubble_start:bubble_end + 1] = 0.0

        return proc_ranges, closest_idx

    def find_max_gap(self, free_space_ranges, angles):
        best_start, best_end = 0, 0
        cur_start            = None
        best_score           = -np.inf

        for i, val in enumerate(free_space_ranges):
            if val > 0.0:
                if cur_start is None:
                    cur_start = i
            else:
                if cur_start is not None:
                    cur_end      = i - 1
                    cur_len      = cur_end - cur_start + 1
                    center_idx   = (cur_start + cur_end) // 2
                    center_angle = abs(angles[center_idx])
                    
                    gap_ranges = free_space_ranges[cur_start:cur_end + 1]
                    max_depth  = np.max(gap_ranges) if len(gap_ranges) > 0 else 0.0

                    score = (cur_len * max_depth) - (300.0 * (center_angle ** 2))

                    if score > best_score:
                        best_score = score
                        best_start = cur_start
                        best_end   = cur_end

                    cur_start = None

        if cur_start is not None:
            cur_end      = len(free_space_ranges) - 1
            cur_len      = cur_end - cur_start + 1
            center_idx   = (cur_start + cur_end) // 2
            center_angle = abs(angles[center_idx])
            
            gap_ranges = free_space_ranges[cur_start:cur_end + 1]
            max_depth  = np.max(gap_ranges) if len(gap_ranges) > 0 else 0.0
            
            score = (cur_len * max_depth) - (300.0 * (center_angle ** 2))

            if score > best_score:
                best_start = cur_start
                best_end   = cur_end

        return best_start, best_end

    def find_best_point(self, start_i, end_i, ranges):
        weight_far    = self.get_parameter('weight_far').get_parameter_value().double_value
        weight_center = self.get_parameter('weight_center').get_parameter_value().double_value

        gap_ranges = ranges[start_i:end_i + 1]
        furthest_i = start_i + int(np.argmax(gap_ranges))
        center_i   = (start_i + end_i) // 2

        best_i = int(round(weight_far * furthest_i + weight_center * center_i))
        best_i = int(np.clip(best_i, start_i, end_i))

        return best_i

    def lidar_callback(self, data: LaserScan):
        angle_min       = data.angle_min
        angle_increment = data.angle_increment
        min_gap_size    = self.get_parameter('min_gap_size').get_parameter_value().integer_value

        proc_ranges = self.preprocess_lidar(data.ranges)

        n      = len(proc_ranges)
        angles = angle_min + np.arange(n) * angle_increment
        proc_ranges[np.abs(angles) > np.radians(90)] = 0.0

        proc_ranges = self.disparity_extender(proc_ranges, angle_increment)

        proc_ranges, _ = self.apply_bubble(proc_ranges, angle_increment)

        start_i, end_i = self.find_max_gap(proc_ranges, angles)

        if end_i - start_i < min_gap_size:
            self.get_logger().warn('Gap too narrow, dropping to crawl speed.')
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed          = 0.1
            self.publisher.publish(drive_msg)
            return

        best_i     = self.find_best_point(start_i, end_i, proc_ranges)
        best_angle = angle_min + best_i * angle_increment

        steer = float(np.clip(best_angle, -0.35, 0.35))

        abs_steer    = abs(steer)
        speed_fast   = self.get_parameter('speed_fast').get_parameter_value().double_value
        speed_medium = self.get_parameter('speed_medium').get_parameter_value().double_value
        speed_slow   = self.get_parameter('speed_slow').get_parameter_value().double_value

        if abs_steer < 0.1:
            velocity = speed_fast
        elif abs_steer < 0.2:
            velocity = speed_medium
        else:
            velocity = speed_slow

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed          = velocity
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()