# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from ... import Twist
from sensor_msgs.msg import Imu
from ... import LaserScan
from ... import Odometry

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
        self.vel_publisher=self.create_publisher(...)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(...)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.subscription = self.create_subscription(IMU, "/imu", self.imu_callback, qos_profile=qos)
        ...
        
        # ENCODER subscription
	self.subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        ...
        
        # LaserScan subscription 
        self.subscription = self.create_subscription(LaserScan, "/laserscan", self.laser_callback, qos_profile=qos)
        ...
        
        self.motion_start_time=self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    timestep_num = 10000
list_imu = [[None]*2]*timestep_num
imu_num = 0
list_odom = [[None]*4]*timestep_num
odom_num = 0
list_scan = [[None]*361]*timestep_num
scan_num = 0


    def imu_callback(self, imu_msg: Imu):
        timestamp = time.from_msg(imu_msg.header.stamp).nanoseconds

        imu_quaternionx = imu_msg.orientation.x
        imu_quaterniony = imu_msg.orientation.y
        imu_quaternionz = imu_msg.orientation.z  
        imu_quaternionw = imu_msg.orientation.w 

        quat = [imu_quaternionx, imu_quaterniony, imu_quaternionz, imu_quaternionz]
        yaw = euler_from_quaternion(quat)
        # print(f'Message Timestamp = {timestamp}')
        # print(f'Current Robot Yaw = {yaw}')

        list_imu[imu_num][0]=timestamp
        list_imu[imu_num][1] = yaw

        imu_num += 1
        
    def odom_callback(self, odom_msg: Odometry):
    
        timestamp = time.from_msg(odom_msg.header.stamp).nanoseconds
        
        odom_orientation = odom_msg.pose.pose.orientation 
        odom_x_pos = odom_msg.pose.pose.position.x
        odom_y_pos = odom_msg.pose.pose.position.y

        
        # pass to a list
        list_odom[odom_num][0] = timestamp
        list_odom[odum_num][1] = odom_orientation
        list_odom[odum_num][2] = odom_x_pos
        list_odom[odum_num][3] = odom_y_pos

        odum_num += 1

    
    # print(f'Message Timestamp = {timestamp}')
    # print(f'Current Robot Orientation = {odom_orientation}')
    # print(f'Current Robot X Position = {odom_x_pos}')
    # print(f'Current Robot Y Position = {odom_y_pos}')
        
        ... # log odom msgs
                
    def laser_callback(self, laser_msg: LaserScan):
        timestamp = time.from_msg(laser_msg.header.stamp).nanoseconds
        
        angle_min = laser_msg.angle_min
        angle_max = laser_msg.angle_max
        angle_increment = laser_msg.angle_increment

        angle = angle_min

        list_scan[scan_num][0]=timestamp


        i = 0
        while angle <= angle_max:
            
            range = laser_msg.ranges[i]

            print(f'Lidar Range  = {range} @ {angle}')
            list_scan[scan_num][i+1] = range
            angle += angle_increment
            i += 1 
        
        scan_num += 1
                
    def timer_callback(self):
        
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
        vel = 0.1
        msg = Twist()
        # fill up the twist msg for line motion
        msg.linear.x = vel
        msg.angular.z = 0.0
        return msg

    def make_circular_twist(self):
        vel = 0.1
        radius = 0.5
        msg = Twist()
        # fill up the twist msg for circular motion
        msg.linear.x = vel
        msg.angular.z = vel/radius
        return msg

    def make_spiral_twist(self):
        vel = 0.1
        init_radius = 0.2
        t = (self.get_clock().now.nanoseconds - self.motion_start_time) * 1e-9
        radius = init_radius + 0.1*t
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
