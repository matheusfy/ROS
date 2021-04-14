#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class TurtleControl:
    def __init__(self):
        rospy.init_node("tb3control_node", anonymous=True)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84
    
    def update_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta =  yaw

    
    def ref_distance(self, ref_pose):
        return np.sqrt(  (ref_pose.x - self.pose.x)**2 + (ref_pose.y - self.pose.y)**2)

    def linear_angular_vel_control(self, ref_pose, kp = 1.5, ka = 6):
        # velocidade linear
        distance = self.ref_distance(ref_pose)
        control = kp* distance
        if abs(control) > self.max_vel:
            control = self.max_vel*np.sign(control)
        # velocidade angular
        angle_r = np.arctan2(ref_pose.y - self.pose.y,  ref_pose.x - self.pose.x ) 
        control_angular = ka*(angle_r - self.pose.theta)         
        if abs(control_angular) > self.max_ang:
            control_angular = self.max_ang*np.sign(control_angular)

        return control, control_angular

    def move2ref(self, x_ref, y_ref):
        ref_pose = Pose()
        ref_pose.x = x_ref
        ref_pose.y = y_ref
        ref_tol = 0.01
        vel_msg = Twist()
        while self.ref_distance(ref_pose) >= ref_tol:
            vel_msg.linear.x, vel_msg.angular.z = self.linear_angular_vel_control(ref_pose)
            
            rospy.loginfo("valor vel_linear : %f", vel_msg.linear.x)
            rospy.loginfo("valor vel_angular: %f", vel_msg.angular.z)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.vel_publisher.publish(vel_msg)

            self.rate.sleep()
            if rospy.is_shutdown():
                break
        # stop
        vel_msg.linear.x = 0
        vel_msg.angular.z= 0
        self.vel_publisher.publish(vel_msg)

        rospy.loginfo("Finished")

if __name__ == '__main__':
    bot = TurtleControl()
    time.sleep(1)
    rospy.loginfo("Insira o valor de x:")
    x = int(input())
    rospy.loginfo("Insira o valor de y:")
    y = int(input())
    bot.move2ref(x,y)
