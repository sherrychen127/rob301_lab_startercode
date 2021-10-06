#!/usr/bin/env python

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
            "color_rgb", UInt32MultiArray, queue_size=1
        )
        self.camera_subscriber = rospy.Subscriber(
            "raspicam_node/image", Image, self.camera_callback, queue_size=1
        )

    def camera_callback(self, data):
        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        array = cv_img[:200]

        intermediate = np.mean(array, axis=0)
        color = np.mean(intermediate, axis=0)
        print("r: {}, g: {}, b: {}".format(color[0], color[1], color[2]))

        # publish the data as an array [r, g, b]
        msg = UInt32MultiArray()
        msg.data = color
        self.color_sensor_publisher.publish(msg)


def main():
    rospy.init_node("camera_rgb")
    camera = CameraRGB()
    rospy.spin()


if __name__ == "__main__":
    main()
