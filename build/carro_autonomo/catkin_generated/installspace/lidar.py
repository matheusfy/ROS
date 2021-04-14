#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose

#Criando classe para o tópico Lidar
#Começar declarando a classe e criando função principais e de inicialização
class Lidar:
    def __init__(self):
        
        # Inicializa Nó
        # @param 1 : Nome do nó
        # @param 2 : Anonimo?
        rospy.init_node('NO_SENSOR_LIDAR', anonymous=True)
        
        # Se inscreve no topico "scan" do robo 0 (mestre)
        # @param 1 : O topico
        # @param 2 : O objeto do Topico (LaserScan)
        # @param 3 : a função de callback
        rospy.Subscriber('/scan', LaserScan, self.update)
        rospy.Subscriber('/cmd_vel,', Twist, self.update_vel)
        rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.pose = Pose()
        self.scan = LaserScan()


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

    def corrige_orientacao(self):
        # Frente do robo 0°
        # Direita do robo de 0 ~(-90°) (4º Quadrante)
        # Diagonal esquerda inferior 90° ~ 180(2º Quadrante)
        # Esquerda robo de 0 ~ 90° (1º Quadrante)

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
    try:
        mestre = Lidar()
        time.sleep(1)
        rospy.loginfo("Insira o valor de x:")
        x = int(input())
        rospy.loginfo("Insira o valor de y:")
        y = int(input())
        mestre.move2ref(x,y)
    except rospy.ROSInterruptException:
        pass 
