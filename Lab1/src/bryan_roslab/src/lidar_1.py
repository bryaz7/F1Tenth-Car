#!/usr/bin/env python
# Subcriber on /scan topic
# Publisher on topics /closest and /farest

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from bryan_roslab.msg import Range
import numpy as np

def callback(msg):
    print("Finding the cloest and farest points in lidar")
    closest = min(msg.ranges)
    farest = max(msg.ranges)

    closest = np.float64(closest)
    farest = np.float64(farest)

    print("Publishing on /closest_point topic")
    print(closest)
    pub_1.publish(closest)

    print("Publishing on /farest_point topic")
    print(farest)
    pub_2.publish(farest)

if __name__=='__main__':
    rospy.init_node('lidar',anonymous=True)

    sub=rospy.Subscriber('/scan', LaserScan, callback)
    pub_1=rospy.Publisher('/closest_point',Range)
    pub_2=rospy.Publisher('/farest_point',Range)
    
    rospy.spin()
