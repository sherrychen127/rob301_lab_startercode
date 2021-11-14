#!/usr/bin/env python

"""
Get mean color (r, g, b) value and publish to /mean_img_rgb
"""

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import UInt32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraRGB(object):
    def __init__(self):
        self.color_sensor_publisher = rospy.Publisher(
            "mean_img_rgb", UInt32MultiArray, queue_size=1
        )
        self.camera_subscriber = rospy.Subscriber(
            "raspicam_node/image", Image, self.camera_callback, queue_size=1
        )

    def camera_callback(self, msg):
        bridge = CvBridge()
        try:
            rgb_cv_img = bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print(e)

        # publish the color sensor reading
        color_array = rgb_cv_img[:200]
        intermediate = np.mean(color_array, axis=0)
        color = np.mean(intermediate, axis=0)

        color_msg = UInt32MultiArray()
        color_msg.data = color
        self.color_sensor_publisher.publish(color_msg)


def main():
    rospy.init_node("camera_rgb")
    camera = CameraRGB()
    rospy.spin()


if __name__ == "__main__":
    main()
