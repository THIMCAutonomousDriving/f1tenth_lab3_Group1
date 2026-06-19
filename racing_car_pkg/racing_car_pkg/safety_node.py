#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np
import math
from std_srvs.srv import Empty

class AEB_node(Node):
    def __init__(self):
        # Initialize node with a name
        super().__init__('safety_node')
        
        self.declare_parameter("sim_or_real", "sim")

        # Initialize the variables for the subscribers/publishers
        self.laser_scan = LaserScan()
        self.odom = Odometry()
        self.ackermann = AckermannDriveStamped()
        self.stop_msg = Bool()
        self.stop = False
        
        self.current_steering_angle = 0.0

        self.subscriber_laser = self.create_subscription(LaserScan, '/scan', self.TTC_calc, 10)
        self.srv = self.create_service(Empty, 'aeb_reset', self.aeb_reset)
        self.publisher_b = self.create_publisher(Bool, '/aeb_stop', 10)

        if self.get_parameter("sim_or_real").get_parameter_value().string_value == 'sim':
            #sim
            self.get_logger().info("Safety Node startet in configuration: simulation")
            self.subsciber_odo = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
            self.publisher_a = self.create_publisher(AckermannDriveStamped, '/drive', 10)
            self.subsciber_drive_gf_sim = self.create_subscription(AckermannDriveStamped, '/drive_gf', self.teleop_callback_Ack, 10)

            ### parameter
            # Define parameter for min TTC definieren (in s)
            self.declare_parameter("min_TTC",0.35)

        else:
            #real
            self.get_logger().info("Safety Node startet in configuration: reality")
            self.subsciber_odo = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            self.publisher_a = self.create_publisher(AckermannDriveStamped, '/drive', 10)
            self.subsciber_drive_gf = self.create_subscription(AckermannDriveStamped, '/drive_gf', self.teleop_callback_Ack, 10) 

            ### parameter
            # Define parameter for min TTC definieren (in s)
            self.declare_parameter("min_TTC",0.5)

    def aeb_reset(self, request, response):
        self.stop = False
        self.stop_msg.data = self.stop
        self.publisher_b.publish(self.stop_msg)
        return response

    def odom_callback(self, msg): 
        self.odom = msg

    def teleop_callback_Ack(self, msg:AckermannDriveStamped):
        self.teleop = msg
        
        self.current_steering_angle = msg.drive.steering_angle
        
        if self.teleop.drive.speed >= 0 and self.stop == True:
            self.ackermann.drive.speed = 0.0
            self.publisher_a.publish(self.ackermann)
        else:
            self.ackermann.drive.speed = self.teleop.drive.speed
            self.ackermann.drive.steering_angle = self.teleop.drive.steering_angle
            self.publisher_a.publish(self.ackermann)

    def teleop_callback_Twist(self, msg:Twist):
        self.teleop = msg
        
        self.current_steering_angle = msg.angular.z
    
        if self.teleop.linear.x >= 0 and self.stop == True:
            self.ackermann.drive.speed = 0.0
            self.publisher_a.publish(self.ackermann)
        else:
            self.ackermann.drive.speed = self.teleop.linear.x
            self.ackermann.drive.steering_angle = self.teleop.angular.z
            self.publisher_a.publish(self.ackermann)

    def TTC_calc(self, msg: LaserScan):
        self.laser_scan = msg
        
        # converting to numpy for easier handling
        self.np_laser_scan = np.array(self.laser_scan.ranges, copy=True) 
        
        ### dealing with inf, nan and out of range values
        self.min_value = 0.15 
        self.max_value = 25 
        
        # checking for inf and nan + replacing it 
        self.np_laser_scan = np.where(np.isfinite(self.np_laser_scan), self.np_laser_scan, self.max_value)
        # replacing everything unrealistically small with the max value
        self.np_laser_scan = np.where(self.np_laser_scan < self.min_value, self.max_value, self.np_laser_scan)
        
        ### Calculating range rate via velocity (Vectorized)
        angles = self.laser_scan.angle_min + self.laser_scan.angle_increment * np.arange(len(self.np_laser_scan))
        linear_x = self.odom.twist.twist.linear.x
        
        self.np_range_rate = np.round(-linear_x * np.cos(angles - self.current_steering_angle), 5)

        # negate the range rate and then cut off the negatives (according to the formula)
        self.np_range_rate = -self.np_range_rate
        self.np_range_rate = np.where(self.np_range_rate <= 0, 0.001, self.np_range_rate) 

        # calculating the TTC
        self.TTC = self.np_laser_scan / self.np_range_rate 
        
        # Evaluating min TTC threshold (Vectorized)
        min_ttc = self.get_parameter('min_TTC').get_parameter_value().double_value
        violations = np.where(self.TTC < min_ttc)[0]

        if len(violations) > 0:
            first_idx = violations[0]
            self.get_logger().info(f"had to break: (TTC was: {self.TTC[first_idx]:.2f})", throttle_duration_sec=1.0)
            self.stop = True
            self.stop_msg.data = self.stop
            self.publisher_b.publish(self.stop_msg)    
        else: 
            if self.stop == False:
                self.stop_msg.data = self.stop
                self.publisher_b.publish(self.stop_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = AEB_node()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()