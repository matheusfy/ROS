https://github.com/jefersonjlima/examples/blob/main/Chapter02/icp_example//icp_pose.py

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import iterative_closest_point

class ICP:
    def __init__(self):
        # load arguments
        self.show_plot =    rospy.get_param('/show_plot')
        self.pose_topic =   rospy.get_param('/pose_topic')
        self.odom_topic =   rospy.get_param('/odom_topic')
        self.laser_topic =  rospy.get_param('/laser_topic')

        rospy.init_node(self.pose_topic, anonymous=True)
        rospy.Subscriber(self.laser_topic, LaserScan, self.ls_callback)
        rospy.Subscriber(self.odom_topic, Odometry,  self.odom_callback)
        self.scan = LaserScan()
        self.odom = Odometry()
        self.rate = rospy.Rate(2)

        self.H_ = np.eye(4)
        self.previous_pcloud = None

        fig = plt.figure(figsize=(12, 6), facecolor='white')
        self.gpose  = fig.add_subplot(122, frameon=False)
        self.lidar  = fig.add_subplot(121, frameon=False, projection='polar')
        plt.show(block=False)

    def ls_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.odom = msg

    def pol2cart(self):
        x = []; y = []
        data = self.scan.ranges
        try:
            for r,deg in zip(data, enumerate(data)):
                if r != np.inf:
                    x.append( r * np.cos( deg[0] * np.pi/180 ))
                    y.append( r * np.sin( deg[0] * np.pi/180 ))
            x = np.asarray(x)
            y = np.asarray(y)
        except:
            x, y = (None, None)
        return x,y

    def run(self):
        while not rospy.is_shutdown():
            x, y = self.pol2cart()
            if self.previous_pcloud is None:
                self.previous_pcloud = np.vstack((x, y))
                T = np.array([0,0])
                R = np.eye(2)

            else:
                current_pcloud = np.vstack((x, y))
                nVector = min(self.previous_pcloud.shape[1], current_pcloud.shape[1])
                _, R, T = iterative_closest_point.icp_matching(self.previous_pcloud[:,:nVector],current_pcloud[:,:nVector])
                self.previous_pcloud = current_pcloud

            # transformation matrix
            H = np.eye(4)
            H[0:2,0:2] = R
            H[0:2,3] = T

            self.H_ = self.H_ @ H

            # http://planning.cs.uiuc.edu/node103.html
            theta = np.rad2deg(np.arctan(self.H_[1,0]/self.H_[0,0]))

            print(np.round(self.H_, 2))
            print(f"theta is {theta} Degrees")

            angles = [i for i in range(0, 360)] 
            angles = np.deg2rad(angles)
            self.lidar.cla()
            self.lidar.plot(angles, self.scan.ranges)

            self.gpose.cla()
            self.gpose.plot(self.H_[0,3], self.H_[1,3], 'r x')
            self.gpose.plot(self.odom.pose.pose.position.x,
                            self.odom.pose.pose.position.y, 'b o')
            self.gpose.set_xlim([-1,1])
            self.gpose.set_ylim([-1,1])
            self.gpose.grid()
            self.gpose.legend(["icp pose","real pose"])
            plt.draw()
            plt.pause(0.1)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        myPose = ICP()
        myPose.run()
    except rospy.ROSInterruptException:
        pass
