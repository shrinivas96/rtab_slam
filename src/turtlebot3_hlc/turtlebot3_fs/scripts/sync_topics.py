#!/usr/bin/env python
import message_filters
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def callback(odom_msg:Odometry, scan_msg:LaserScan):
    filePath = "/home/ivengar/workspace/rtab_slam/src/turtlebot3_hlc/turtlebot3_fs/scripts/logger.dat"
    scan_data = "L {} {}\n".format(scan_msg.header.stamp.secs, list(scan_msg.ranges))
    odom_data = "O {} {}, {}, {}, {}, {}, {}, {}\n".format(odom_msg.header.stamp.secs, odom_msg.pose.pose.position.x,
                                                           odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
                                                           odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                                                           odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
    
    data = scan_data + odom_data

    with open(filePath, 'a') as logger:
        logger.write(data)

    # rospy.loginfo("{}".format())


def main():
    rospy.init_node("combinerOP")
    print("Started node")

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