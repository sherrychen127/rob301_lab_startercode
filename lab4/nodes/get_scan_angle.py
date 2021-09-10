#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np

def get_scan():
        scan = rospy.wait_for_message('scan', LaserScan)
       
        scan_filter = []
        for val in scan.ranges:
            if val == 0:
                scan_filter.append(30)
            else:
                scan_filter.append(val)


        in_range = np.array(scan_filter[0:180]) #only use the scan where y>=0 
        angles = np.array(range(0,180))*np.pi/180
        y_range = np.copy(in_range)*np.sin(np.copy(angles)) #computes the y-distance to each measurement
#        print(x_range)
        
        angles = np.where(np.logical_and(np.logical_and(y_range <=0.65,y_range>=0.5)==1, in_range < 3)) #filter out measurements with ranges larger than 1.2m or y_range less than 0.5m
#        print(angles)
#        print(in_range[angles])
        median_angle = np.median(angles)

    	return median_angle

def main():
    rospy.init_node('get_scan_angle')
    scan_pub = rospy.Publisher('scan_angle', String, queue_size=1)
    try:
    	while(1):
        	ind = get_scan()
        	scan_pub.publish(str(ind))
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
