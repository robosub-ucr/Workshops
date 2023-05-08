# subscribes to forward_cam and processes the image

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import buoy_detector

# create CV bridge
bridge = CvBridge()

# just for debugging: publisher for the processed image (because rqt is easy to use)
img_pub = rospy.Publisher("obstacle", Image)

# callback for detecting an obstacle
def detect_obstacle(ros_img):
    img = bridge.imgmsg_to_cv2(ros_img, desired_encoding='rgb8')
    
    # find a buoy
    buoy_found, buoy_location, debug_img = buoy_detector.detect_buoy(img)

    if buoy_found:
        print("Buoy in sight!", buoy_location)
    
    # you would also publish what obstacles are currently seen and their locations

    # for debugging, publish it (mono8 is greyscale)
    ros_img = bridge.cv2_to_imgmsg(debug_img, encoding='mono8')
    img_pub.publish(ros_img)


def camera_processer():
    # create the subscriber
    subscriber = rospy.Subscriber("forward_cam", Image, detect_obstacle)

    # create the node
    rospy.init_node('obstacle_detection')

    # run until end
    rospy.spin()


if __name__ == "__main__":
    try:
        camera_processer()
    except rospy.ROSInterruptException:
        pass