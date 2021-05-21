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

        self.pose = Pose()
        self.vel = Twist()
        self.scan = LaserScan()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84
        self.previous_points = None

        # variavel professor np.eye cria uma matriz diagonal de dimensao x
        self.H_ = np.eye(4)

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

    '''
    th_matrix = matriz de rotação do sistema de referência B em A pela 
                posição de Bp
    '''

    def tf(angulo, S, Q, *args, **kargs):
        if angulo == 'deg':
            angulo = np.deg2rad(angulo)
        th_matrix = np.array([[np.cos(angulo),   -np.sin(angulo), 0.,      Q[0]],
                            [np.sin(angulo),   np.cos(angulo),  0.,      Q[1]],
                            [0.,        0.,                   1.,      Q[2]],
                            [0.,        0.,                   0.,      S]])
        return th_matrix

    def pt_cartesiano(self):
        x1=[]; y1=[]
        lidar = np.asarray(self.scan.ranges)
        lidar[np.isinf(lidar)==True] = 3.5
        for r,deg in zip(lidar,enumerate(lidar)):
            if r!= 3.5:
                x1.append( r*np.cos(deg[0]* np.pi/180))
                y1.append( r*np.sin(deg[0]* np.pi/180))
        x1 = np.asarray(x1)
        y1 = np.asarray(y1)
        return x1,y1

    def theta_objetivo(self, ref_x, ref_y):
        angle_correcao = np.arctan2(ref_y - self.pose.y,  ref_x - self.pose.x )
        return angle_correcao
        

    def main(self, ref_x, ref_y):
        time.sleep(2)
        Q = np.array([ref_x, ref_y, 0.0])
        S = 1.0
        # Diferenca entre angulo do objetivo e a orientação robo
        angulo = self.theta_objetivo(ref_x=ref_x, ref_y=ref_y)
        # Posição do robo em relação aos eixos cartesiano
        x, y = self.pt_cartesiano()

        len_points = len(x)
        #print(x)
        m_points = np.array([ [x[0]], [y[0]], [0], [1]])
        
        for i in range(1, len_points):
            m_points = np.hstack((m_points, np.array([ [ x[i]], [y[i]], [0], [1]] )))
        deg = 'deg'
        saida = self.tf(angulo, S, Q, angle = deg ) @ m_points

        plt.figure()
        plt.plot(x,y, 'b *')
        plt.plot(saida[0,:], saida[1,:], 'g *')
        plt.show()
        coord_x = 0
        coord_y = 0
        theta = 0
        input("pause: ")
        while True:

            x, y = self.pt_cartesiano()
            # Baseando no codigo do professor
            
            if self.previous_points == None:
                self.previous_points = np.vstack((x, y))
                
                T = np.array([0,0]) 
                R = np.eye(2) 
            else:
                current_points = np.vstack((x,y))
                nVector = min(self.previous_points.shape[1], current_points.shape[1])
                R ,T = icp_example.icp_matching(self.previous_points[:,:nVector], current_points[:,:nVector])
                self.previous_points = current_points
            
            H = np.eye(4)
            H[0:2,0:2]= R
            H[0:2,3] = T

            self.H_ = self.H @ H


            #     # Tentativa 1
            # if self.vel.linear.x > 0:

            #     x_atual, y_atual = self.pt_cartesiano()
            #     current_points = np.vstack((x_atual, y_atual))


            #     R,T = icp_example.matching(self.previous_points, current_points)
                
            #     coord_x = coord_x + self.vel.linear.x*T*np.cos(self.pose.theta)
            #     coord_y = coord_y + self.vel.angular.z*T*np.sin(self.pose.theta)
            #     theta = theta + self.vel.angular.z*T
            #     # plt.figure()
            #     # plt.plot(x, y, '.')
            #     # plt.show()
            #     print("vel linear atual: " + str(self.vel.linear.x) )
            #     time.sleep(2)
            if rospy.is_shutdown():
                break
            # for x1, y1 in zip(x,y):
            #     print("x: %d, y: %d" % (x1, y1))
        

'''
    Bpx, Bpy, Bpz, 1 = m_points
    theta = angulo deslocamento em relacao a orientacao inicial
    Q = deslocamento [x_ref,y_ref,z_ref=0, S=1] 

'''

if __name__ == '__main__':
    try:
        mestre = LidarICP()

        mestre.main(ref_x =2, ref_y=2)

    except rospy.ROSInterruptException:
        pass 

        