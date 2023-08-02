import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

flag = 0

def convert_depth_image(ros_image):
    global flag
    cv_bridge = CvBridge()
    try:
        depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)
    depth_array = np.array(depth_image, dtype=np.float32)
    np.save("depth_img.npy", depth_array)
    rospy.loginfo(depth_array)
    #To save image as png
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imwrite("/home/ivengar/workspace/rtab_slam/src/turtlebot3_hlc/turtlebot3_laser_deg/scripts/depth_img.png", depth_colormap)

    rospy.loginfo("Written!!!")
    rospy.Subscriber.unregister()
    #Or you use 
    # depth_array = depth_array.astype(np.uint16)
    # cv2.imwrite("depth_img.png", depth_array)


def pixel2depth():
    rospy.loginfo("In node")
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback=convert_depth_image, queue_size=10)
    # r = rospy.Rate(10)
    # while flag == 0 and not rospy.is_shutdown():
    #     rospy.loginfo("Waiting for image to be published.")
    #     r.sleep()
    rospy.spin()

if __name__ == '__main__':
    pixel2depth()