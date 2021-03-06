#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Point

class swarm_control:

    def __init__(self):
        rospy.init_node('swarm_control_tb3',  anonymous=False) 
        rospy.Subscriber('cmd_vel', Twist, self.update)
        self.pub1 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        

        self.new_cmd = Twist()
        self.cmd = Twist()

        self.rate = rospy.Rate(10) # Hz
        rospy.loginfo("/cmd_vel >> /tb3_0/cmd_vel")
        

 

    def update(self, msg):
        self.cmd = msg

    def run(self):

        while not rospy.is_shutdown():

            self.new_cmd.linear.x = self.cmd.linear.x
            self.new_cmd.angular.z = self.cmd.angular.z

            self.pub1.publish(self.new_cmd)
            

            self.rate.sleep()


        

if __name__ == '__main__':
    try:
        swarm_tb3 = swarm_control()
        swarm_tb3.run()

    except rospy.ROSInterruptException:
        pass

