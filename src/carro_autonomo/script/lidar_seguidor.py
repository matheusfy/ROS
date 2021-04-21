#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import time
import numpy as np 

class Lidar_seguidor:
    def __init__(self):
        rospy.init_node('Lidar_seguidor', anonymous=True)
        
        rospy.Subscriber('/tb3_1/cmd_vel', Twist, self.update_vel)
        rospy.Subscriber('/tb3_1/scan', LaserScan, self.update)
        rospy.Subscriber('/tb3_1/odom', Odometry, self.update_pose)
        self.vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        
        self.pose = Pose()
        self.scan = LaserScan()
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84

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

        # pega referencia inicial do mestre
        # 2. Se a diferença da imagem inicial do lidar com a imagem atual for > 0.5 -> pega referencia e converte em x,y
    def update_vel(self, msg):
        self.vel = msg

    def corrige_orientacao(self, ref_pose, ka=1):
        # Frente do robo 0°
        # Direita do robo de 0 ~(-90°) (4º Quadrante)
        # Diagonal esquerda inferior 90° ~ 180(2º Quadrante)
        # Esquerda robo de 0 ~ 90° (1º Quadrante)
        angle_r = np.arctan2(ref_pose.y - self.pose.y,  ref_pose.x - self.pose.x ) + 30 # deslocado em 30° 
        control_angular = ka*(angle_r - self.pose.theta)         
        if abs(control_angular) > self.max_ang:
            control_angular = self.max_ang*np.sign(control_angular) + 2*np.pi*0.15 # teoricamente rotaciona para o angulo da direcao do ponto + 45°
        return control_angular

    def corrige_orientacao_final(self, ref_pose, ka=1):

        angle_r = np.arctan2(ref_pose.y - self.pose.y,  self.pose.y )
        control_angular = ka*(angle_r - self.pose.theta)         
        if abs(control_angular) > self.max_ang:
            control_angular = self.max_ang*np.sign(control_angular) 
        return control_angular

    def linear_angular_vel_control(self, ref_pose, kp = 1, ka = 3):
        # velocidade linear
        distance = self.ref_distance(ref_pose)
        control = kp* distance
        if abs(control) > self.max_vel:
            control = self.max_vel*np.sign(control)
        
        # velocidade angular
        angle_r = np.arctan2(ref_pose.y - self.pose.y,  ref_pose.x - self.pose.x ) 
        print("angulo orientacao do robo: " + str(angle_r)) 
        print("orientacao: "+ str(self.pose.theta))
        control_angular = ka*(angle_r - self.pose.theta) 

        if abs(control_angular) > self.max_ang:
            control_angular = self.max_ang*np.sign(control_angular)
        
        return control, control_angular 
    def ref_distance(self, ref_pose):
        return np.sqrt(  (ref_pose.x - self.pose.x)**2 + (ref_pose.y - self.pose.y)**2)

    def ref_mestre(self):
        
        previous_lidar = np.asarray(self.scan.ranges)
        previous_lidar[previous_lidar==np.inf] = 3.5
        current_lidar = np.asarray(self.scan.ranges)
        current_lidar[current_lidar==np.inf] = 3.5
        diff_lidar = previous_lidar - current_lidar
        diff_lidar = np.abs(diff_lidar)
        print(previous_lidar)
        while np.amax(np.abs(diff_lidar)) < 0.5:
            time.sleep(3)
            current_lidar = np.asarray(self.scan.ranges)
            current_lidar[current_lidar==np.inf] = 3.5
            diff_lidar = np.abs(previous_lidar - current_lidar)
            # print(diff_lidar)
            # print("aqui")

        # print(diff_lidar)
        for r,deg,diff in zip(previous_lidar, enumerate(previous_lidar), diff_lidar):
            if diff >=1.5:
                # print("distancia %.1f: ", r)
                # print("angulo: %.1f ", deg[0])

                x = r*np.cos(deg[0]*np.pi/180)
                y = r*np.sin(deg[0]*np.pi/180)

        return x,y

    def move2ref(self):
        ref_pose = Pose()
        ref_tol = 0.1
        vel_msg = Twist()
        while True:
                
            ref_pose.x, ref_pose.y = self.ref_mestre()
            print("x: %d, y: %d", ref_pose.x, ref_pose.y)
            # Enquanto mestre não chega no ponto solicitado via comando.:
            # input()
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
                
                self.vel_pub.publish(vel_msg)

                self.rate.sleep()
                if rospy.is_shutdown():
                    break
            # stop
            vel_msg.linear.x = 0
            vel_msg.angular.z= 0
            self.vel_pub.publish(vel_msg)
            print("orientacao: "+ str(self.pose.theta))
            while self.pose.theta>= 0.1:
                # print("orientacao: "+ str(self.pose.theta))
                vel_msg.angular.z = self.corrige_orientacao_final(ref_pose)
                vel_msg.linear.x = 0
                
                self.vel_pub.publish(vel_msg)

            vel_msg.linear.x = 0
            vel_msg.angular.z= 0
            self.vel_pub.publish(vel_msg)
        
            rospy.loginfo("waiting mestre move")



if __name__== '__main__':
    try:
        seguidor = Lidar_seguidor()
        time.sleep(5)
        # print(seguidor.scan.ranges)
        # mapa = np.asarray(seguidor.scan.ranges)
        # mapa[mapa==np.inf] = 0
        # for r,deg in zip(mapa, enumerate(mapa)):
        #     if r!=0:
        #         x = r*np.cos(deg[0]*np.pi/180)
        #         y = r*np.sin(deg[0]*np.pi/180)

        # print("x: %.1f" %x)
        # print("y: %.1f"% y)        
        seguidor.move2ref()
        #seguidor.move2ref()

    except rospy.ROSInterruptException:
        pass




