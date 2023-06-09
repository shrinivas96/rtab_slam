#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import Odometry

def plot_x(odom_msg):
    global counter
    if counter % 10 == 0:
        stamp = odom_msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter", anonymous=True)
    rospy.Subscriber("/odom", Odometry, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()