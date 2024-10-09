# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']



class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.subscription = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        self.imu_initialized = True
        
        # ENCODER subscription
        self.subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        self.odom_initialized = True
        
        # LaserScan subscription 
        self.subscription = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile=qos)
        self.laser_initialized = True

        self.motion_start_time=self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.timer_callback)

        #boolean to ensure that only one timestamp of lidar scan is plotted to an excel file 
        self.laser_logged = False


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py
    def imu_callback(self, imu_msg: Imu):
        #gets timestamp from robot in nanoseconds 
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        #x and y accelerations
        acc_x = imu_msg.linear_acceleration.x 
        acc_y = imu_msg.linear_acceleration.y

        # yaw rate
        angular_z = imu_msg.angular_velocity.z
        
        #logging imu values
        imu_list = [acc_x, acc_y, angular_z, timestamp]
        self.imu_logger.log_values(imu_list)

    def odom_callback(self, odom_msg: Odometry):

        #timestamp in nanoseconds
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        
        #reading the values for the quaternion of the robot's movement
        odom_quaternionx = odom_msg.pose.pose.orientation.x
        odom_quaterniony = odom_msg.pose.pose.orientation.y
        odom_quaternionz = odom_msg.pose.pose.orientation.z
        odom_quaternionw = odom_msg.pose.pose.orientation.w


        # creating a vector quaternion
        quat = [odom_quaternionx,odom_quaterniony, odom_quaternionz, odom_quaternionw]

        #reads orientation, x position, and y position of the robot
        odom_orientation = euler_from_quaternion(quat)
        odom_x_pos = odom_msg.pose.pose.position.x
        odom_y_pos = odom_msg.pose.pose.position.y

        #log odom values
        odom_list = [ odom_x_pos, odom_y_pos,odom_orientation, timestamp]
        self.odom_logger.log_values(odom_list)
                
    def laser_callback(self, laser_msg: LaserScan):
        #function only runs once, below logic sets laser_logged variable to true 
        if self.laser_logged:
            return
        self.laser_logged = True

        #timestamp in nanoseconds
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        
        #reads angle_min, angle_max, and angle_increment values for laser scanner
        angle_increment = laser_msg.angle_increment

        #for loop that goes through range data and logs range values after each angle increment
        for rng in laser_msg.ranges:
            laser_list = [rng, angle_increment, timestamp]
            self.laser_logger.log_values(laser_list)

    

                
    def timer_callback(self):
        # print('TIMER CALLBACK')
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
        
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot
    
    def make_acc_line_twist(self):
        print('SETTING VEL')
        vel = 0.3
        msg = Twist()
        # fill up the twist msg for line motion
        msg.linear.x = vel
        msg.angular.z = 0.0
        return msg

    def make_circular_twist(self):
        vel = 0.3
        radius = 0.1
        msg = Twist()
        # fill up the twist msg for circular motion
        msg.linear.x = vel
        msg.angular.z = vel/radius
        return msg

    def make_spiral_twist(self):
        vel = 0.3
        init_radius = 0.1
        t = (self.get_clock().now().nanoseconds - self.motion_start_time) * 1e-9
        radius = init_radius + 0.01*t
        msg=Twist()
        # fill up the twist msg for spiral motion
        msg.linear.x = vel
        msg.angular.z = vel/radius
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")

