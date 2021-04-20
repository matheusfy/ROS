#!/usr/bin/env python

import rospy
from nav_msgs import Odometry
from geometry_msgs import Twist
from geometry_msgs import Pose
from sensor_msgs import LaserScan


class Lidar_seguidor:
    def __init__(self):

        rospy.init_node('Lidar_seguidor', anonymous=True)
        rospy.Subscriber('/tb3_1/scan', LaserScan, self.update)
        rospy.Subscriber('/tb3_1/odom', Odometry, self.update_pose)
        self.vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.pose = Pose()
        self.scan = LaserScan()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22 # vel linear maxima
        self.max_ang = 2.84 # vel angular maxima

    def update(self, msg):
        #Funcao que pega os pontos do lidar
        self.scan = msg

    def update_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # angulo em relacao ao meu referencial inicial 
        self.pose.theta =  yaw   


if __name__= '__main__':


