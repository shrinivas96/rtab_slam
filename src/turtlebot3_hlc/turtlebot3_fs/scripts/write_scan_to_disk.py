#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def callback(scan_msg:LaserScan, filePath):
    # filePath = "/home/ivengar/workspace/rtab_slam/src/turtlebot3_hlc/turtlebot3_fs/scripts/logger.dat"
    scan_data = "{}\n".format(list(scan_msg.ranges))
    
    data = scan_data

    with open(filePath, 'a') as logger:
        logger.write(data)


def main():
    rospy.init_node("disk_write_scan")
    myargv = rospy.myargv(argv=sys.argv)
    rospy.Subscriber(name='/sim_scan', data_class=LaserScan, callback=callback, callback_args=myargv[1])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()