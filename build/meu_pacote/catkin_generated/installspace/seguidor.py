#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import time

# rodar rosrun teleop para utilizar awsd
# rodar rosrun swarm_control para publicar no Mestre o comando de movimento
# rodar seguidor.py para fazer o seguidor seguir o mestre se movendo


class SeguidorTurtle:
    def __init__(self):
        # cria topico
        rospy.init_node("no_seguidor", anonymous=False)
        
        # se inscreve no topico de odometria do waffle 1
        rospy.Subscriber("/tb3_0/odom", Odometry, self.update_pose0)
        
        # se inscreve no topico de odometria do waffle 1
        rospy.Subscriber("/tb3_1/odom", Odometry, self.update_pose1)
        #self.vel_pub0 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        
        self.vel_pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10) # objeto para publicar no topico de velocidade do seguidor
        self.pose0 = Pose() # posicao do mestre
        self.pose1 = Pose() # posicao do seguidor
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84 

    #funcao para pegar posicao do mestre
    def update_pose0(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_,_,yaw) = euler_from_quaternion(orientation_list)
        self.pose0.x = msg.pose.pose.position.x
        self.pose0.y = msg.pose.pose.position.y
        self.pose0.theta = yaw
        #rospy.loginfo("direcao yaw: " + str(self.pose0.theta))
    
    #funcao para pegar posição do seguidor
    def update_pose1(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_,_,yaw) = euler_from_quaternion(orientation_list)
        self.pose1.x = msg.pose.pose.position.x
        self.pose1.y = msg.pose.pose.position.y
        self.pose1.theta = yaw

    # retorna a distância entre mestre e seguidor
    def ref_distance(self):
        return np.sqrt(  (self.pose0.x - self.pose1.x)**2 + (self.pose0.y - self.pose1.y)**2)

    # calcula a velocidade linear que ira enviar para o seguidor
    def linear_vel_control(self, kp = 1.5):
        distance = self.ref_distance()
        control = kp* distance
        if abs(control) > self.max_vel:
            control = self.max_vel*np.sign(control)
        return control

    # calcula a velocidade angula que ira enviar para o seguidor (rotaciona)
    def angular_vel_control(self, kp=6):
        angle_r = np.arctan2(self.pose0.y - self.pose1.y,  self.pose0.x - self.pose1.x )        
        control = kp*(angle_r - self.pose1.theta)
        if abs(control) > self.max_ang:
            control = self.max_ang*np.sign(control)
        return control

    #funcao que atualiza o seguidor para seguir ou parar
    def move2ref(self):

        ref_tol = 1 # tolerancia de distancia entre mestre e seguidor
        vel_msg = Twist() #objeto que publica mensagem que atualiza a velocidade 
        flag = False
        rospy.loginfo("Começando ")
        #while self.ref_distance() >= ref_tol:
        while not rospy.is_shutdown():
            if self.ref_distance() >= ref_tol:
                rospy.loginfo(self.ref_distance())
                vel_msg.linear.x = self.linear_vel_control()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel_control()

                self.vel_pub1.publish(vel_msg)

                self.rate.sleep()
                flag = False
            else:
                # Quando a distancia entre mestre e seguidor é menor que a tolerância -> para o seguidor   
                if flag == False:
                    rospy.loginfo("Perto demais do mestre, desativando o motor do seguidor")
                    vel_msg.linear.x = 0
                    vel_msg.angular.z= 0
                    self.vel_pub1.publish(vel_msg)
                    flag = True
        rospy.loginfo("Finished")

if __name__ == '__main__':
    try:
        pose = SeguidorTurtle()
        pose.move2ref()
    except rospy.ROSInterruptException:
        pass