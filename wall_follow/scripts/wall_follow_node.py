#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        # Initialized Publisher for the new drive data
        self.publisher_ackermann = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.laser_scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        
        # Updated PID gains based on tuning advice for smoother cornering
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("ki", 0.0) 
        self.declare_parameter("kd", 0.1)

        self.declare_parameter("desired_distance", 0.6)
        self.declare_parameter("angle_diff", 45.0)
        # Reduced lookahead so the car turns closer to the actual corner
        self.declare_parameter("lookahead", 0.2) 

        self.declare_parameter("lor", 'left')
        
        # Store history
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0

        self.time = 0.0
        self.prev_time = 0.0


    def get_range(self, range_data: LaserScan, angle): 

        angle_increment = range_data.angle_increment                    # single step in rad
        min_angle = range_data.angle_min                                # starting angle of lidar
        index = int(round((angle - min_angle) / angle_increment , 0))   # calc index by finding the needed steps for that angle
        
        # Safety check: ensure index is within the bounds of the array
        if index < 0 or index >= len(range_data.ranges):
            return 0.0

        range_val = range_data.ranges[index]                            # get the range at that point
        
        # check for inf and nan
        if not np.isfinite(range_val):
            return 0.0

        return range_val 

    def get_error(self, range_data: LaserScan, dist): 

        if self.get_parameter("lor").get_parameter_value().string_value == 'left':
            self.v = 1
        else:                   # check for left or right wall following. 
            self.v = -1

        theta = np.deg2rad(self.get_parameter("angle_diff").get_parameter_value().double_value)
        b_angle = np.deg2rad(90.0 * self.v) # 90 deg angle, if we want to follow the right wall, make it -90
        a_angle = b_angle - theta * self.v  # 45 deg angle, for right wall, we have to do -90 -(-45)

        a = self.get_range(range_data, a_angle) # distance to whats front left / right (prob. wall too)
        b = self.get_range(range_data, b_angle) # distance to whats directly on the left / right (wall)

        # safety check: returns 0.0 if Nan, inf, or empty
        if not np.isfinite(a) or not np.isfinite(b) or a == 0.0 or b == 0.0:
            return 0.0

        alpha = np.arctan2(a * np.cos(theta) - b, # estimated wall angle relative to the car
                        a * np.sin(theta)) 

        Dt = b * np.cos(alpha) # curent distance to wall
        L = self.get_parameter('lookahead').get_parameter_value().double_value  # lookahead distance
        
        Dt1 = Dt + L * np.sin(alpha) # future projected distance to wall (estimated future distance)

        # positive error means car is too far from the wall
        error = Dt1 - dist # actual d - desired d

        return error

    def pid_control(self, error, velocity, range_data: LaserScan):

        self.time = range_data.header.stamp.sec * 1e9 + range_data.header.stamp.nanosec

        # CRITICAL FIX: Prevent the first-frame delta-time explosion
        if self.prev_time == 0.0:
            self.prev_time = self.time
            self.prev_error = error
            return # Skip the first control loop to initialize previous values properly

        self.kp =  self.get_parameter('kp').get_parameter_value().double_value
        self.ki =  self.get_parameter('ki').get_parameter_value().double_value
        self.kd =  self.get_parameter('kd').get_parameter_value().double_value

        dt = (self.time - self.prev_time) / 1e9 # Delta time in seconds
        
        # Safety check: prevent division by zero if simulator publishes two scans instantly
        if dt <= 0.0:
            dt = 0.01

        # P-Part
        p = self.kp * error
        
        # I-Part
        self.integral += error * dt
        i = self.ki * self.integral 

        # D-Part
        d = self.kd * ((error - self.prev_error) / dt)

        # Combination
        pid = p + i + d
        
        # CRITICAL FIX: Clamp the steering angle to realistic physical limits (~0.35 radians)
        MAX_STEERING_ANGLE = 0.35
        angle = max(min(pid, MAX_STEERING_ANGLE), -MAX_STEERING_ANGLE)

        # Optional: Print to console to monitor the newly clamped values vs the raw PID
        # self.get_logger().info(f"pid: {pid:.2f}, clamped angle: {angle:.2f}, p: {p:.2f}, i: {i:.2f}, d: {d:.2f}")

        # Create AckermannDrive and fill it with angle and velocity then publish
        drive_msg = AckermannDriveStamped()
        
        drive_msg.drive.steering_angle = angle * self.v
        drive_msg.drive.speed = velocity

        self.publisher_ackermann.publish(drive_msg)

        # Store history
        self.prev_time = self.time
        self.prev_error = error


    def scan_callback(self, msg):

        # The desired distance to follow 
        self.desired_distance =  self.get_parameter('desired_distance').get_parameter_value().double_value
        
        self.error = self.get_error(msg, self.desired_distance) 

        # Create a dynamic velocity profile based on the magnitude of the error
        abs_error = abs(self.error)

        if abs_error < 0.1:
            # Error is very low, the car is parallel to the wall, speed up
            velocity = 1.5 
        elif abs_error < 0.3:
            # Slight deviation, medium speed
            velocity = 1.0 
        else:
            # Large error, sharp turn or correction required, slow down
            velocity = 0.5 

        # Trigger the PID controller with the calculated error and velocity
        self.pid_control(self.error, velocity, msg)
        
def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()