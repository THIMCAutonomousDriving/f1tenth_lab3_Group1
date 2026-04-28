#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math


class AEB_node(Node):
    def __init__(self):
        # Initialize node with a name
        super().__init__('aeb')
        
        ### parameter
        # Define parameter for min TTC definieren (in s)
        self.declare_parameter("min_TTC",0.7)

        # Initialize the variables for the subscribers/publishers
        self.laser_scan = LaserScan()
        self.odom = Odometry()
        self.ackermann = AckermannDriveStamped()
        self.stop = False
        ### subscriber and publisher
        # Subscriber for laser scan
        self.subscriber_laser = self.create_subscription(LaserScan, '/scan', self.TTC_calc, 10)

        # Subscriber for odometry
        self.subsciber_odo = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10) # sim
        #self.subsciber_odo = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # reality
        

        # Publisher for Ackermann speed 
        self.publisher_a = self.create_publisher(AckermannDriveStamped, '/drive', 10) # sim
        #self.publisher_a = self.create_publisher(AckermannDriveStamped, '/teleop_aeb', 10) # reality
    
        self.subsciber_teleop = self.create_subscription(AckermannDriveStamped, '/cmd_vel', self.teleop_callback, 10) # sim
        #self.subsciber_teleop = self.create_subscription(AckermannDriveStamped, '/teleop', self.teleop_callback, 10) # reality


    
    def odom_callback(self, msg): # aus odo subscriber
        # save the received odom message into our own variable that we can access anywhere now
        self.odom = msg


    def teleop_callback(self, msg:Twist):
        self.teleop = msg
        if self.teleop.linear.x >= 0 and self.stop == True:
            self.ackermann.drive.speed = 0.0
            self.publisher_a.publish(self.ackermann)
        else:
            self.stop = False
            self.publisher_a.publish(self.teleop)


    def TTC_calc(self, msg: LaserScan):
        self.laser_scan = msg
        
        # converting to numpy for easier handling
        self.np_range_rate = np.array(self.laser_scan.ranges, copy=True) # initializing range rate with same length, the values will be overwritten,
        self.np_laser_scan = np.array(self.laser_scan.ranges, copy=True) # Convert the array to np so it can be calculated easier
        #important that the copy is done like this otherwise copy and original arrays will be changed at the same time
        

        ### dealing with inf, nan and out of range values
        self.min_value = 0.01 # too small value, that would have to be a mistake, so we will set it to a max range (haS to be checked with the real car)
        self.max_value = 25 # biggest value, that could realistically occur (we set all the mistakes to this value, so we wont run into problems when calculating while also not accidentally braking)
        
        # checking for inf and nan + replacing it 
        self.np_laser_scan = np.where(np.isfinite(self.np_laser_scan), self.np_laser_scan, self.max_value)
        # replacing everything unrealistically small with the max value
        self.np_laser_scan = np.where(self.np_laser_scan < self.min_value, self.max_value, self.np_laser_scan)
        

        ### Calculating range rate via velocity
        for i in range (len(self.np_range_rate)):
            # range rate = - linear vel * cos (angle)
            self.np_range_rate[i] = round(- self.odom.twist.twist.linear.x * math.cos(self.laser_scan.angle_min + self.laser_scan.angle_increment * i), 5)

        #negate the range rate and then cut off the negatives (according to the formula)
        for i in range (len(self.np_range_rate)):
            self.np_range_rate[i] = -self.np_range_rate[i]
            if self.np_range_rate[i] <= 0:
                self.np_range_rate[i] = 0.001 # just so it isnt 0 and the ttc therefore inf

        # calculating the TTC
        self.TTC = self.np_laser_scan / self.np_range_rate 


        for i in range (len(self.TTC)):
            if self.TTC[i] < self.get_parameter('min_TTC').get_parameter_value().double_value:
                self.get_logger().info(f"had to break: (TTC was: {self.TTC[i]:.2f})", throttle_duration_sec=1.0)
                #if self.ackermann.drive.speed = 0.0
                self.stop = True

                self.ackermann.drive.speed = 0.0
                self.publisher_a.publish(self.ackermann)       # do this here once, so its immediate

                


def main(args=None):
    rclpy.init(args=args)
    aeb = AEB_node()
    rclpy.spin(aeb)
    aeb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()