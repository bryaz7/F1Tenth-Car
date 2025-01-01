#!/usr/bin/env python
# Subcriber on /scan topic
# Publisher on /scan_range topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from bryan_roslab.msg import scan_range

def callback(msg):
    print("Finding the cloest and farest points in lidar")
    range_msg = scan_range()
    range_msg.closest = min(msg.ranges)
    range_msg.farest = max(msg.ranges)

    print("Publishing on /scan_range topic")
    print(range_msg)
    pub.publish(range_msg)


if __name__=='__main__':
    rospy.init_node('lidar',anonymous=True)

    sub=rospy.Subscriber('/scan', LaserScan, callback)
    pub=rospy.Publisher('/scan_range',scan_range)
    
    rospy.spin()
