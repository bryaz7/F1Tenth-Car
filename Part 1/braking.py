#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from ackermann_msgs.msg import AckermannDriveStamped
import math

class ClosestFarthestPointFinder:
    def __init__(self):
        rospy.init_node('closest_farthest_point_finder')
        
        self.closest_point_pub = rospy.Publisher('/closest_point', Float64, queue_size=10)
        self.farthest_point_pub = rospy.Publisher('/farthest_point', Float64, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=0)
      
        self.speed = 0

        self.brake_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=0)
        self.brake_bool_publisher = rospy.Publisher("brake_bool", Bool, queue_size=0)

        # Manually define the TTC threshold parameter
        self.ttc_threshold = rospy.get_param("~ttc_threshold", 3.0)

    def scan_callback(self, msg):
        closest_point = min(msg.ranges)
        farthest_point = max(msg.ranges)
        
        self.closest_point_pub.publish(closest_point)
        self.farthest_point_pub.publish(farthest_point)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = -0.65
        self.drive_publisher.publish(drive_msg)
        rospy.loginfo("Closest Point: %f meters, Farthest Point: %f meters" % (closest_point, farthest_point))

        brake = False
        '''for i, range_val in enumerate(msg.ranges):
            if range_val != float('inf') and range_val > 0: 
                v_i = self.speed * math.cos(msg.angle_min + i * msg.angle_increment)
                if v_i > 0:
                    ttc = range_val / v_i
                    
                    if ttc < self.ttc_threshold:
                    #if closest_point < 0.3:
                        brake = True
                        brake_msg = AckermannDriveStamped()
                        brake_msg.header.stamp = rospy.Time.now()
                        brake_msg.drive.speed = 0
                        self.brake_publisher.publish(brake_msg)
                        rospy.loginfo(brake_msg)
                        break

        brake_bool_msg = Bool()
        brake_bool_msg.data = brake
        self.brake_bool_publisher.publish(brake_bool_msg)'''
        
        for i, range_val in enumerate(msg.ranges):
        	if(closest_point<0.5):
                    brake = True
                    brake_msg = AckermannDriveStamped()
                    brake_msg.header.stamp = rospy.Time.now()
                    brake_msg.drive.speed = 0
                    self.brake_publisher.publish(brake_msg)
                    rospy.loginfo(brake_msg)
                    break	
        brake_bool_msg = Bool()
        brake_bool_msg.data = brake
        self.brake_bool_publisher.publish(brake_bool_msg)
        
        
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ClosestFarthestPointFinder()
        node.run()
    except rospy.ROSInterruptException:
        pass
