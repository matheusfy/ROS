#!usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import numpy as np
import matplotlib.pyplot as plt


class TransRotation:

    def __init__(self):
        
        rospy.init_node('No_TransRotation', anonymous= True)

        rospy.Subscriber('/cmd_vel', Twist, self.update_vel)
        rospy.Subscriber('/scan', LaserScan, self.update)
        rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose = Pose()
        self.scan = LaserScan()
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84

    def update(self, msg):
        self.scan = msg

    def update_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # angulo em relacao ao meu referencial inicial 
        self.pose.theta =  yaw   

    def update_vel(self, msg):
        self.vel = msg

    def Rotaciona(self, mapa, theta, angle = 'rad'):
        x1 = []; y1 =[]
        
        for r,deg in zip(mapa, enumerate(mapa)):
            if r != 3.5:
                x1.append(r * np.cos( deg[0] * np.pi/180 ))
                y1.append(r * np.sin( deg[0] * np.pi/180 ))
        
        if angle == 'deg':
            theta = np.deg2rad(theta)
        
        rotate_matrix = np.array([[np.cos(theta), -np.sin(theta),0], 
        [ np.sin(theta), np.cos(theta), 0], [0,0,1]])
        
        return x1, y1, rotate_matrix
    
    def Lidar(self):
        ref_360 = np.asarray(self.scan.ranges)
        # ref_360[np.isinf(ref_360)==True] = 3.5
        return ref_360

if __name__ == '__main__':
    try:
        Rotacao_Translacao = TransRotation()

        theta = float(input("Digite o angulo em Graus que deseja rotacionar: "))
        mapa = Rotacao_Translacao.Lidar()
        x1, y1, matriz_rotacao = Rotacao_Translacao.Rotaciona(mapa, theta, angle='deg')

        x,y = x1, y1
        len_points = len(x)
        m_points = np.array([[x[0]], [y[0]], [0]])
        for i in range(1, len_points):
            m_points = np.hstack((m_points,  np.array([[ x[i]],[  y[i] ],[0.5]])))

        out = matriz_rotacao @ m_points
        
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize = (6*3,6))
        ax1.plot(x,y, '.')
        ax1.plot(0,0, 'b o')
        ax1.legend(["Original"])
        ax2.plot(out[0,:], out[1,:],'.')
        ax2.plot(0,0, 'b o')
        ax2.legend(["Rotation"])
        ax3.plot(x,y, '.')
        ax3.plot(out[0,:], out[1,:],'.')
        ax3.plot(0,0, 'b o')
        ax3.legend(["Original","Rotation"])
        plt.show()

    except rospy.ROSInterruptException:
        pass 

        
        