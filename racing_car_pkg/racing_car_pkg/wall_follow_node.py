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
        drive_topic = '/drive_wf'
        
        # Initializing Publisher for the new drive data
        self.publisher_ackermann = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Initializing Subsciber for the LiDAR data
        self.laser_scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # Initializing the Parameter to choose which wall to follow (Standard is left wall)
        self.declare_parameter("lor", 'left')
        
        # Initializing the Paramter for differentiating betwen simulation or real configurations (Stardard is simulation)
        self.declare_parameter("sim_or_real", "sim")
        
        # Initializing all the relevant Parameter in their configuration
        if self.get_parameter("sim_or_real").get_parameter_value().string_value == 'sim':
            self.declare_parameter("kp",1.3)
            self.declare_parameter("ki",0.25)
            self.declare_parameter("kd",1.5)

            self.declare_parameter("desired_distance",1.0)
            self.declare_parameter("angle_diff", 45.0)
            self.declare_parameter("lookahead", 0.9)

            self.get_logger().info('ReactiveFollowGap node initialized in sim mode.')

        else:
            self.declare_parameter("kp",1.25)
            self.declare_parameter("ki",0.1)
            self.declare_parameter("kd",0.1)

            self.declare_parameter("desired_distance",0.4)
            self.declare_parameter("angle_diff", 60.0)
            self.declare_parameter("lookahead", 1.0)

            self.get_logger().info('ReactiveFollowGap node initialized in real mode.')


        
        # Intitialiting the "history" variables
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0

        self.time = 0.0
        self.prev_time = 0.0

    def get_range(self, range_data: LaserScan, angle):

        angle_increment = range_data.angle_increment                    # single step in deg
        min_angle = range_data.angle_min                                # starting angle of lidar
        index = int(round((angle - min_angle) / angle_increment , 0))   # calc index by finding the needed steps for that angle
        range = range_data.ranges[index]                                # get the range at that point
        
        # Check for inf and nan
        if not np.isfinite(range):
            return 0.0

        return range 

    def get_error(self, range_data: LaserScan, dist):

        # Check wether to follow the left or right wall
        if self.get_parameter("lor").get_parameter_value().string_value == 'left':
            self.v = 1
        else:                  
            self.v = -1

        theta = np.deg2rad(self.get_parameter("angle_diff").get_parameter_value().double_value)
        #theta = np.deg2rad(45.0*)
        b_angle = np.deg2rad(90.0 * self.v) #90 deg angle, if we want to follow the right wall, make it -90
        a_angle = b_angle - theta * self.v  #45 deg angle, for right wall, we have to do -90 -(-45)

        a = self.get_range(range_data, a_angle) #distance to whats front left / right (prob. wall too)
        b = self.get_range(range_data, b_angle) #distance to whats directly on the left / right (wall)

        alpha = np.arctan2(a * np.cos(theta) - b, # calculation of the heading angle
                        a * np.sin(theta)) 

        Dt = b * np.cos(alpha) # calculation fo the curent distance to the wall
        
        ### Debug logger for theta, a, b, and alpha
        #self.get_logger().info(f"theta: {theta:.2f}; alpha {alpha:.2f}, a: {a:.2f}; b {b:.2f}") 
        

        L = self.get_parameter('lookahead').get_parameter_value().double_value  # lookahead distance

        Dt1 = Dt + L * np.sin(alpha) # calculation of the future projected distance to the wall


        error = Dt1 - dist # calculation the error (distance between the future and desired distance)
        # positive error means car is too far from the wall

        return error

    def pid_control(self, error, range_data: LaserScan):

        # Getting the time from the LiDAR (in nanoseconds)
        self.time = range_data.header.stamp.nanosec

        pid = 0.0 # initialize the pid angle

        self.kp =  self.get_parameter('kp').get_parameter_value().double_value
        self.ki =  self.get_parameter('ki').get_parameter_value().double_value
        self.kd =  self.get_parameter('kd').get_parameter_value().double_value

        ### P-Part
        # Calculated as: kp * error
        p = self.kp*error
        
        ### I-Part
        # Calculated as: integral + ki * error * delta_time
        i = self.integral + self.ki * error * (self.time - self.prev_time) / 1e9    #1e9 is converting nanoseconds to seconds        
        self.integral = i #Integral is the accumulated correction

        # Anti-windup filter
        if self.integral >= 0.5:
            self.integral = 0.5
        if self.integral <= -0.5:
            self.integral = -0.5

        ### D-Part
        # Calculated as: kd * (delta_error / delta_time)
        d = self.kd * ((error - self.prev_error) / (self.time - self.prev_time) *1e9)   #1e9 is converting nanoseconds to seconds

        # Summation of the PID
        pid = p + i + d

        ### Debug logger for the PID
        #self.get_logger().info(f"pid was: {pid:.2f}, p was: {p:.2f}, i was: {i:.2f}, d was: {d:.2f},")

        # Saving the PID as an angle, to simplify understanding
        angle = pid

        #Store history
        self.prev_time = self.time
        self.prev_error = error

        return angle


    def scan_callback(self, msg):

        # The desired distance to follow (e.g., stay 1.0 meters away from the left wall)
        self.desired_distance =  self.get_parameter('desired_distance').get_parameter_value().double_value
        
        # Calculate the the error
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
        
        # Trigger the PID controller with the calculated error
        angle = self.pid_control(self.error, msg)

        ### Publishing 
        #Create AckermannDrive and fill it with angle and velocity then publish
        drive_msg = AckermannDriveStamped()

        drive_msg.drive.steering_angle = angle * self.v     #variable v enables following both walls 
        drive_msg.drive.speed = velocity

        ### Debug logger for the velocity and the steering angle
        #self.get_logger().info(f"Desired velocity set to: {velocity:.2f}; Angle corrected to {angle:.2f}")

        self.publisher_ackermann.publish(drive_msg)

        
def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()