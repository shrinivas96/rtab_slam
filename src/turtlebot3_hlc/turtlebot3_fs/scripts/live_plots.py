#!/usr/bin/env python 
import sys
import rospy
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry

def plot_x(odom_msg):
    global counter
    if counter % 10 == 0:
        stamp = odom_msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, '*')
        plt.title("Plotting {}".format(topic_to_plot))
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    topic_to_plot = sys.argv[1]

    rospy.logerr("Currently received to plot: {}".format(topic_to_plot))
        
    counter = 0
    rospy.init_node("plotter", anonymous=True)
    rospy.Subscriber(name=topic_to_plot, data_class=Odometry, callback=plot_x)
    plt.ion()
    plt.show()
    rospy.spin()