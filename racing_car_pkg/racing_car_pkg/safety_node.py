#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
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
        self.stop = False


        ### subscriber and publisher

        # Subscriber for laser scan
        self.subscriber_laser = self.create_subscription(LaserScan, '/scan', self.TTC_calc, 10)

        #service zum zurücksetzten des bremsstatus
        self.srv = self.create_service(Empty, 'aeb_reset', self.aeb_reset)

        if self.get_parameter("sim_or_real").get_parameter_value().string_value == 'sim':

            self.get_logger().info("Safety Node startet in configuration: simulation")
            # Subscriber for odometry
            self.subsciber_odo = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10) # sim

            # Publisher for Ackermann speed 
            self.publisher_a = self.create_publisher(AckermannDriveStamped, '/drive', 10) # sim

            # subscriber for command topic that we let through or not
            # command for teleop_key: 
            # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_key
            self.subsciber_teleop_key = self.create_subscription(Twist, '/teleop_key', self.teleop_callback_Twist, 10) # sim        
            self.subsciber_drive_wf_sim = self.create_subscription(AckermannDriveStamped, '/drive_wf', self.teleop_callback_Ack, 10) # wall follower
            self.subsciber_drive_gf_sim = self.create_subscription(AckermannDriveStamped, '/drive_gf', self.teleop_callback_Ack, 10) # gap follower

            ### parameter
            # Define parameter for min TTC definieren (in s)
            self.declare_parameter("min_TTC",0.35)

        else:
            self.get_logger().info("Safety Node startet in configuration: reality")
            # Subscriber for odometry
            self.subsciber_odo = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # reality

            # Publisher for Ackermann speed 
            self.publisher_a = self.create_publisher(AckermannDriveStamped, '/drive', 10) # reality

            # subscriber for command topic that we let through or not
            self.subsciber_drive_wf = self.create_subscription(AckermannDriveStamped, '/drive_wf', self.teleop_callback_Ack, 10) 
            self.subsciber_drive_gf = self.create_subscription(AckermannDriveStamped, '/drive_gf', self.teleop_callback_Ack, 10) 
            self.subsciber_teleop = self.create_subscription(AckermannDriveStamped, '/teleop', self.teleop_callback_Ack, 10)

            ### parameter
            # Define parameter for min TTC definieren (in s)
            self.declare_parameter("min_TTC",0.4)

    def aeb_reset(self, request, response):
        self.stop = False
        return response

    def odom_callback(self, msg): # aus odom subscriber
        # save the received odom message into our own variable that we can access anywhere now
        self.odom = msg

    def teleop_callback_Ack(self, msg:AckermannDriveStamped):
        #self.get_logger().info(f"Recieved Ackermann: {msg})", throttle_duration_sec=5.0)
        self.teleop = msg
        if self.teleop.drive.speed >= 0 and self.stop == True:
            self.ackermann.drive.speed = 0.0
            self.publisher_a.publish(self.ackermann)
        else:
            #self.stop = False # testweise raus
            self.ackermann.drive.speed = self.teleop.drive.speed
            self.ackermann.drive.steering_angle = self.teleop.drive.steering_angle
            self.publisher_a.publish(self.ackermann)

    def teleop_callback_Twist(self, msg:Twist):
        self.teleop = msg
    
        if self.teleop.linear.x >= 0 and self.stop == True:
            self.ackermann.drive.speed = 0.0
            self.publisher_a.publish(self.ackermann)
        else:
            #self.stop = False # testweise raus
            self.ackermann.drive.speed = self.teleop.linear.x
            self.ackermann.drive.steering_angle = self.teleop.angular.z
            self.publisher_a.publish(self.ackermann)


    def TTC_calc(self, msg: LaserScan):
        self.laser_scan = msg
        
        # converting to numpy for easier handling
        self.np_range_rate = np.array(self.laser_scan.ranges, copy=True) # initializing range rate with same length, the values will be overwritten,
        self.np_laser_scan = np.array(self.laser_scan.ranges, copy=True) # Convert the array to np so it can be calculated easier
        #important that the copy is done like this otherwise copy and original arrays will be changed at the same time
        

        ### dealing with inf, nan and out of range values
        self.min_value = 0.15 # too small value, that would have to be a mistake, so we will set it to a max range (haS to be checked with the real car)
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
        #self.get_logger().info(f"(Range_Rate was: {self.np_range_rate})")
        #self.get_logger().info(f"(TTC was: {self.TTC})", throttle_duration_sec=1.0)

        for i in range (len(self.TTC)):
            if self.TTC[i] < self.get_parameter('min_TTC').get_parameter_value().double_value:
                self.get_logger().info(f"had to break: (TTC was: {self.TTC[i]:.2f})", throttle_duration_sec=1.0)
                self.stop = True
                self.ackermann.drive.speed = 0.0
                self.publisher_a.publish(self.ackermann)       # do this here once, so its immediate


def main(args=None):
    rclpy.init(args=args)
    safety_node = AEB_node()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()