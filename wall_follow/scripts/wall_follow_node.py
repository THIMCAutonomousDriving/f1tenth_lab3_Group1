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

        # TODO: create subscribers and publishers
        
        #Initialized Publisher for the new drive data
        self.publisher_ackermann = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 

        # TODO: store history
        # self.integral = 
        # self.prev_error = 
        # self.error = 

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle): #alex and fiona
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR       full laser scan message please
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        # angle_increment = self.laser_scan.angle_increment
        # min_angle = self.laser_scan.min_angle
        # index = round((angle - min_angle) / angle_increment , 0)
        # range = laser_scan.ranges[index]

        #TODO: implement
        return 0.0 # range

    def get_error(self, range_data, dist): #others
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        return 0.0

    def pid_control(self, error, velocity): #alex and fiona
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
# TODO: Use kp, ki & kd to implement a PID controller 
        #As far as I understand it right now the programm will continously use this function and this serves as the loop
        #Still need to integrate time and prev_time
        #The calculation for the PID right now uses the "global"-Variable for error/prev_error.
        #Troubleshooting may include switchitg "self.error" to this functions "error"
        pid = 0.0

        # P-Part
        #Calculated as: kp * error
        p = self.kp*error
        
        # I-Part
        #Calculated as: integral + ki * error * delta_time
        i = self.integral + self.ki * error * (time - prev_time)
        self.integral = i #Integral is the accumulated correction/The actual integratet part up until now
        #If necessary implement Wind up filter here.
        #in Inicializing: self.integral = 0.0

        # D-Part
        #Calculated as: kd * (delta_error / delta_time)
        d = self.kd * ((self.error - self.prev_error) / (time - prev_time))

        # Combination
        pid = p + i + d
        
        angle = 0.0
        angle = pid

        #Create AckermannDrive and fill it with anlge and velocity then publish
        drive_msg = AckermannDriveStamped()
        
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity

        self.get_logger().info(f"Desired velocity set to: {velocity:.2f}; Angle corrected to {angle:.2f}")
        self.publisher_ackermann.publish(drive_msg)
        #Initialize: self.publisher_ackermann = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        #!!If not done in error function: Implement the saving of the previous error: self.prev_error = error
        #!!If someone else needs the time/prev_time otherwise ill need to get it from somewhere.
        #For example laserscan. i. e. laser_scan.scan_time or odometry.header.time.sec / odometry.header.time.nanosec
        #Maybe other function like using real time something might be possible as well

    def scan_callback(self, msg): #others
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = 0.0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


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