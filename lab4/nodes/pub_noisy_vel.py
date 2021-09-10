#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MakeNoise(object):
    def __init__(self, mu=0, std=1e-5):
        self.mu = mu
        self.std = std

        self.measurements = []
        self.measured_state = []
        self.states = []
        self.P_history = []


        self.prev= rospy.Time.now().to_sec()

        self.cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel_noisy', Twist, queue_size = 1)


	
    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x
        twist = Twist()
        twist.linear.x = self.x + np.random.normal(loc=self.mu, scale = self.std)
        self.cmd_pub.publish(twist)


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('noisyvel')
    #cmd_publisher=rospy.Publisher('cmd_vel', Twist, queue_size=1)
    try:
        mu_noise = -0.001
        std_noise = 0.001
        kf = MakeNoise(mu=mu_noise, std=std_noise)
        rospy.spin()
    except:
        print(e)

    finally:
        print("exiting")