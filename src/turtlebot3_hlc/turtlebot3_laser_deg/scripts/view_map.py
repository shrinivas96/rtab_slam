#!/usr/bin/env python
import rospy
import numpy as np
from occupancy_grid_python.occupancy_grid_impl import OccupancyGridManager
from nav_msgs.msg import OccupancyGrid

def main():
    rospy.init_node("map_viz")
    rospy.loginfo("Started node")

    ogm = OccupancyGridManager('/map')

    rospy.loginfo(ogm.origin)

    save = False
    if save:
        rospy.loginfo("Saving data")
        np.savez('/home/ivengar/workspace/rtab_slam/very_unique_ogm.npz', ogm._grid_data)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()