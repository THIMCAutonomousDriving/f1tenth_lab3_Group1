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
        self.kp = 0.5
        self.kd = 0.7
        self.ki = 1.0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0

        self.time = 0.0
        self.prev_time = 0.0

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data: LaserScan, angle): #alex and fiona
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR       full laser scan message please
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        angle_increment = range_data.angle_increment
        min_angle = range_data.angle_min
        index = round((angle - min_angle) / angle_increment , 0)
        range = range_data.ranges[index]

        # check for inf and nan
        if not np.isfinite(range):
            return 0.0

        #TODO: implement
        return range # 

    def get_error(self, range_data, dist): #others
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        theta = np.deg2rad(45.0) # angle between the two beams

        b_angle = np.deg2rad(90.0) #90 deg angle 
        a_angle = b_angle - theta   #45 deg angle

        a = self.get_range(range_data, a_angle) #distance to whats front left (prob. wall too)
        b = self.get_range(range_data, b_angle) #distance to whats directly on the left (left wall)

        # safety check: returns False if Nan, inf 
        if not np.isfinite(a) or not np.isfinite(b):
            return 0.0


        alpha = np.arctan2(a * np.cos(theta) - b, # estimated wall angle relative to the car
                        a * np.sin(theta))


        Dt = b * np.cos(alpha) # curent distance to wall

        L = 1.0 # lookahead distance

        
        Dt1 = Dt + L * np.sin(alpha) # future projected distance to wall (estimated future distance)

        # for left wall following: positive error means car is too far from the left wall
        error = Dt1 - dist # actual d - desired d

        return error

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
        i = self.integral + self.ki * error * (self.time - self.prev_time)
        self.integral = i #Integral is the accumulated correction/The actual integratet part up until now
        #If necessary implement Wind up filter here.
        #in Inicializing: self.integral = 0.0

        # D-Part
        #Calculated as: kd * (delta_error / delta_time)
        d = self.kd * ((self.error - self.prev_error) / (self.time - self.prev_time))

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
        self.prev_time = self.time
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
        # The desired distance to follow (e.g., stay 1.0 meter away from the left wall)
        desired_distance = 1.0 

        error = self.get_error(msg, desired_distance) 

        # Create a dynamic velocity profile based on the magnitude of the error

        abs_error = abs(error)

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
        self.pid_control(error, velocity)
        
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