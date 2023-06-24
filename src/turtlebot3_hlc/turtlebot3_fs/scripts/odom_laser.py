#!/usr/bin/env python
import message_filters
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def callback(odom_msg:Odometry, scan_msg:LaserScan):
    odomFile = "/home/ivengar/workspace/rtab_slam/src/turtlebot3_hlc/turtlebot3_fs/scripts/logOdom.dat"
    scanFile = "/home/ivengar/workspace/rtab_slam/src/turtlebot3_hlc/turtlebot3_fs/scripts/logScan.dat"

    scan_data = "{}\n".format(scan_msg)
    odom_data = "{}\n".format(odom_msg)
    
    with open(odomFile, 'a') as logger:
        logger.write(odom_data)
    
    with open(scanFile, 'a') as logger1:
        logger1.write(scan_data)



def main():
    rospy.init_node("get_laser_odom")
    rospy.loginfo("Started record node")

    odomSub = message_filters.Subscriber('odom', Odometry)
    scanSub = message_filters.Subscriber('scan', LaserScan)

    ts = message_filters.ApproximateTimeSynchronizer([odomSub, scanSub], 1, 1)
    ts.registerCallback(callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()