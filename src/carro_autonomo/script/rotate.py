#!usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

PI = 3.141592
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82


def rotate():
    rospy.init_node('rotate', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # rotacionando o robo
    
    vel_rotacao = float(input("Digite a velocidade de rotacao (deg/s): ") )
    while vel_rotacao > WAFFLE_MAX_ANG_VEL:
        print("ultrapassou limite maximo de rotacao do modelo Waffle. Digite outra velocidade de rotacao.")
        vel_rotacao = float(input("Digite a velocidade de rotacao (deg/s): ") )
    angulo = float(input("Digite quantos graus dejesa rotacionar: "))

    #convertendo para radiano
    vel_angular_rad = vel_rotacao*2*PI/360
    angulo_rad = angulo*2*PI/360
    
    # não usaremos termos linear do robo e nem a rotacao em X e Y
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = abs(vel_rotacao)

    t_init = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < angulo_rad):
        vel_pub.publish(vel_msg)
        t_end = rospy.Time.now().to_sec()
        current_angle = vel_angular_rad*(t_end - t_init)
        rospy.loginfo("angulo atual: %f", current_angle)
    #Para o robo de rotacionar
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)
    #rospy.spin()

if __name__ == '__main__':
    
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass

