#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def callback(odom_msg:Odometry, filePath):
    # filePath = "/home/ivengar/workspace/rtab_slam/src/turtlebot3_hlc/turtlebot3_fs/scripts/log_odom_self.dat"
    odom_data = "{}, {}, {}, {}, {}, {}, {}, {}\n".format(odom_msg.header.stamp.secs, odom_msg.pose.pose.position.x,
                                                           odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
                                                           odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                                                           odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
    
    data = odom_data

    with open(filePath, 'a') as logger:
        logger.write(data)

    # rospy.loginfo("{}".format())


if __name__ == "__main__":
    rospy.init_node("disk_write")
    myargv = rospy.myargv(argv=sys.argv)
    rospy.Subscriber(name='/odom', data_class=Odometry, callback=callback, callback_args=myargv[1])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
