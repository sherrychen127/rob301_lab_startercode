#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32


class Controller(object):
    def __init__(self):
        self._cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )
        """
        complete the function
        """

    def camera_callback(self, data):
        """
        complete the function
        """
        pass

    def follow_the_line(self):
        """
        complete the function
        """
        pass


if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line()
