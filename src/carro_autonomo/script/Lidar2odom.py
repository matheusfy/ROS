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


class Lidar2Odom:
    def __init__(self):

        rospy.init_node('Lidar2Odom', anonymous=True)

        rospy.Subscriber('/scan', LaserScan, self.update)
        rospy.Subscriber('/cmd_vel', Twist, self.update_vel)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose = Pose()
        self.scan = LaserScan()
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
    
    
    def update_vel(self,msg):
        self.msg.linear.x = msg.linear.x
        self.msg.angula.z = msg.angular.z


    #Corrige orientação do robo para desviar do objeto

    def corrige_orientacao(self, ref_pose, ka=1):
        # Frente do robo 0°
        # Direita do robo de 0 ~(-90°) (4º Quadrante)
        # Diagonal esquerda inferior 90° ~ 180(2º Quadrante)
        # Esquerda robo de 0 ~ 90° (1º Quadrante)


        angle_r = np.arctan2(ref_pose.y - self.pose.y,  ref_pose.x - self.pose.x ) + 30 # deslocado em 30° 
        control_angular = ka*(angle_r - self.pose.theta)         
        if abs(control_angular) > self.max_ang:
            control_angular = self.max_ang*np.sign(control_angular) + 2*PI*0.15 # teoricamente rotaciona para o angulo da direcao do ponto + 45°
        return control_angular

    def ref_distance(self, ref_pose):
        return np.sqrt(  (ref_pose.x - self.pose.x)**2 + (ref_pose.y - self.pose.y)**2)

    def linear_angular_vel_control(self, ref_pose, kp = 1, ka = 3):
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
        ref_tol = 0.15
        vel_msg = Twist()
        self.lidar()

        # Enquanto mestre não chega no ponto solicitado via comando.:
        while self.ref_distance(ref_pose) >= ref_tol:
            
            # Pega referência do lidar para detectar objeto a frente
            ref_360 = np.asarray(self.scan.ranges)
            ref_360[np.isinf(ref_360)==True] = 3.5
            
            if ref_360[0] == 3.5:
                v_msgx, v_msgz = self.linear_angular_vel_control(ref_pose)
                vel_msg.linear.x = v_msgx
                vel_msg.angular.z = v_msgz
            elif (ref_360[0] != 3.5) and ref_360[0]>2:
                vel_msg.linear.x = 0.1*self.max_vel
                print("Objeto a frente! mas longe. Andando pra frente lentamente.. ")            
            elif ref_360[0]<= 2:
                vel_msg.linear.x = 0.5*self.max_vel
                vel_msg.angular.z = self.corrige_orientacao(ref_pose)
                print("rotacionando para desviar do objeto")


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
        self.lidar()
        rospy.loginfo("Finished")

    def pt_cartesiano(self):
        x1=[]; y1=[]
        lidar = np.asarray(self.scan.ranges)
        lidar[np.isinf(lidar)==True] = 3.5
        for r,deg in zip(lidar,enumerate(lidar)):
            if r!= 3.5:
                x1.append( r*np.cos(deg[0]* np.pi/180))
                y1.append( r*np.sin(deg[0]* np.pi/180))
        return x1,y1


    def lidar(self):

        x_init, y_init = self.pt_cartesiano()
        plt.figure()
        plt.plot(x_init, y_init, '.')
        plt.show()
        # for x1, y1 in zip(x_init,y_init):
        #     print("x: %d, y: %d" % (x1, y1))
        

if __name__ == '__main__':
    try:
        mestre = Lidar2Odom()
        
        rospy.loginfo("Insira o valor de x:")
        x = int(input())
        rospy.loginfo("Insira o valor de y:")
        y = int(input())
        mestre.move2ref(x,y)

    except rospy.ROSInterruptException:
        pass 

        