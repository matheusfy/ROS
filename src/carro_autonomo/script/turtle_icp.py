#!usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import time
import numpy as np 
import icp_example
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose


class LidarICP:
    def __init__(self):

        rospy.init_node('turtle_icp', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.update_vel)
        rospy.Subscriber('/scan', LaserScan, self.update)
        rospy.Subscriber('/odom', Odometry, self.update_pose)

        sefl.pose = Pose()
        self.vel = Twist()
        self.scan = LaserScan()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84

    def update(self, msg):
        self.scan = msg

    def update_vel(self, msg):
        self.vel = msg

    def update_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # angulo em relacao ao meu referencial inicial 
        self.pose.theta =  yaw  


    def pt_cartesiano(self):
        x1=[]; y1=[]
        lidar = np.asarray(self.scan.ranges)
        lidar[np.isinf(lidar)==True] = 3.5
        for r,deg in zip(lidar,enumerate(lidar)):
            if r!= 3.5:
                x1.append( r*np.cos(deg[0]* np.pi/180))
                y1.append( r*np.sin(deg[0]* np.pi/180))
        return x1,y1

    def theta_objetivo(self, ref_pose):
        angle_correcao = np.arctan2(ref_pose.y - self.pose.y,  ref_pose.x - self.pose.x ) + 30 # deslocado em 30° 
        return angle_correcao      
    

    def lidar(self):
        while True:
            coord_x = 0
            coord_y = 0
            theta = 0
            x_init, y_init = self.pt_cartesiano()
            previous_points = np.vstack((x_init, y_init))
            if self.vel.linear.x > 0:
                x_atual, y_atual = self.pt_cartesiano()
                current_points = np.vstack((x_atual, y_atual))
                R,T = icp_example.matching(previous_points, current_points)
                
                coord_x = coord_x + self.vel.linear.x*T*np.cos(self.pose.theta)
                coord_y = coord_y + self.vel.angular.z*T*np.sin(self.pose.theta)
                theta = theta + self.vel.angular.z*T
                # plt.figure()
                # plt.plot(x_init, y_init, '.')
                # plt.show()
                print("vel linear atual: " + str(self.vel.linear.x) )
                time.sleep(2)
            if rospy.is_shutdown():
                break
            # for x1, y1 in zip(x_init,y_init):
            #     print("x: %d, y: %d" % (x1, y1))
        

if __name__ == '__main__':
    try:
        mestre = LidarICP()
   
        mestre.lidar()

    except rospy.ROSInterruptException:
        pass 

        