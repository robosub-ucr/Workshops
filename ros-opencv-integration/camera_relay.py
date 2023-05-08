# ROS node to relay the webcam feed to a ROS topic

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_relay():
    # create the camera
    cam = cv2.VideoCapture(0)

    # create the bridge
    bridge = CvBridge()

    # create the publisher
    camera_pub = rospy.Publisher('forward_cam', Image, queue_size = 10)

    # create the node
    rospy.init_node("camera_relay")
    rate = rospy.Rate(10)

    print("Relaying camera feed")

    # while the node is active, pull the camera image and publish it
    while not rospy.is_shutdown():
        # get the camera image
        ret, frame = cam.read()
        
        # if getting the image was successful, publish it
        if ret:
            ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            camera_pub.publish(ros_img)

        rate.sleep()


if __name__ == "__main__":
    try:
        camera_relay()
    except rospy.ROSInterruptException:
        pass
