#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import time


class SeguidorTurtle:
    def __init__(self):
        rospy.init_node("no_seguidor", anonymous=False)
        rospy.Subscriber("/tb3_0/odom", Odometry, self.update_pose0)
        rospy.Subscriber("/tb3_1/odom", Odometry, self.update_pose1)
        #self.vel_pub0 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        self.ver_pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.pose0 = Pose()
        self.pose1 = Pose()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84

    def update_pose0(self, msg):
        orientacao_q = msg.pose.pose.orientacao_q
        orientacao_list = [orientacao_x, orientacao_y, orientacao_z, orientacao_w]
        (_,_,yaw) = euler_from_quaternion(orientacao_list)
        self.pose0.x = msg.pose.pose.position.x
        self.pose0.y = msg.pose.pose.position.y
        self.pose0.theta = yaw
    
    def update_pose1(self, msg):
        orientacao_q = msg.pose.pose.orientacao_q
        orientacao_list = [orientacao_x, orientacao_y, orientacao_z, orientacao_w]
        (_,_,yaw) = euler_from_quaternion(orientacao_list)
        self.pose1.x = msg.pose.pose.position.x
        self.pose1.y = msg.pose.pose.position.y
        self.pose1.theta = yaw


    def ref_distance(self):
        return np.sqrt(  (self.pose0.x - self.pose1.x)**2 + (self.pose0.y - self.pose1.y)**2)

    def linear_vel_control(self, kp = 1.5):
        distance = self.ref_distance()
        control = kp* distance
        if abs(control) > self.max_vel:
            control = self.max_vel*np.sign(control)
        return control

    def angular_vel_control(self, kp=6):
        angle_r = np.arctan2(self.pose0.y - self.pose1.y,  self.pose0.x - self.pose1.x )        
        control = kp*(angle_r - self.pose1.theta)
        if abs(control) > self.max_ang:
            control = self.max_ang*np.sign(control)
        return control

    def move2ref(self):

        ref_tol = 0.01
        vel_msg = Twist()
        while self.ref_distance() >= ref_tol:
            vel_msg.linear.x = self.linear_vel_control()
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel_control()

            self.vel_pub1.publish(vel_msg)

            self.rate.sleep()

        # stop
        vel_msg.linear.x = 0
        vel_msg.angular.z= 0
        self.vel_publisher.publish(vel_msg)

        rospy.loginfo("Finished")