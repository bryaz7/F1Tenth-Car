#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class reactive_follow_gap:
    def __init__(self):
        # Topics & Subscriptions,Publishers
        lidarscan_topic = "/scan"
        drive_topic = "/nav"

        self.lidar_sub = rospy.Subscriber(
            lidarscan_topic, LaserScan, self.lidar_callback
        )  # TODO
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=0
        )  # TODO

    def preprocess_lidar_data(self, lidar_ranges):
        """Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
        """
        processed_ranges = list(lidar_ranges)
        window_size = 10
        index = 0

        # Replace each value with the mean of its window
        while index < len(lidar_ranges):
            window_values = processed_ranges[index : index + window_size]
            mean_value = np.mean(window_values)
            processed_ranges[index : index + window_size] = [mean_value] * window_size
            index += window_size

        # Limit the values to a range between 0 and 10
        processed_ranges = [10 if value > 10 else value for value in processed_ranges]
        processed_ranges = [0 if value < 0.8 else value for value in processed_ranges]

        return processed_ranges

    def find_largest_gap(self, lidar_ranges, threshold):
        """Return the start index & end index of the largest gap in lidar_ranges"""
        largest_gap_length = 0
        largest_gap_end = 0
        current_gap_length = 0
        current_gap_start = 0

        for i, range_value in enumerate(lidar_ranges):
            if range_value > threshold:
                current_gap_length += 1
            else:
                if current_gap_length > largest_gap_length:
                    largest_gap_length = current_gap_length
                    largest_gap_end = i - 1
                current_gap_length = 0
                current_gap_start = i + 1

        # Check if the last gap is the largest
        if current_gap_length > largest_gap_length:
            largest_gap_length = current_gap_length
            largest_gap_end = len(lidar_ranges) - 1

        # If no gap is found, decrease the threshold and try again
        if largest_gap_length == 0:
            threshold -= 0.3
            return self.find_largest_gap(lidar_ranges, threshold)

        largest_gap_start = largest_gap_end - largest_gap_length + 1
        return largest_gap_start, largest_gap_end

    def find_furthest_point(self, start_index, end_index, lidar_ranges):
        """Start_index & end_index are start and end indices of the largest gap in lidar_ranges, respectively.
        Return index of the furthest point in lidar_ranges within this gap.
        """

        gap_ranges = lidar_ranges[start_index : end_index + 1]
        furthest_range = max(gap_ranges)
        furthest_point_index_in_gap = gap_ranges.index(furthest_range)
        furthest_point_index = start_index + furthest_point_index_in_gap
        return furthest_point_index

    def lidar_callback(self, data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        lidar_ranges = list(data.ranges)
        start_index = int(math.radians(90) / data.angle_increment)
        end_index = int((math.radians(270) / data.angle_increment)) + 1
        lidar_ranges = lidar_ranges[start_index:end_index]
        processed_ranges = self.preprocess_lidar_data(lidar_ranges)

        # Find the closest point to LiDAR
        closest_point_index = lidar_ranges.index(min(lidar_ranges))

        # Eliminate all points inside 'bubble' (set them to zero)
        bubble_radius = 10
        for i in range(
            closest_point_index - bubble_radius, closest_point_index + bubble_radius
        ):
            processed_ranges[i] = 0

        # Find the largest gap
        gap_start_index, gap_end_index = self.find_largest_gap(processed_ranges, 3)

        # Find the furthest point in the gap
        furthest_point_index = self.find_furthest_point(
            gap_start_index, gap_end_index, lidar_ranges
        )

        # Calculate the steering angle
        steering_angle = furthest_point_index * data.angle_increment - math.pi / 2

        # Determine the speed based on the steering angle
        if 0 <= abs(math.degrees(steering_angle)) <= 10:
            speed = 0.4
        elif 10 < abs(math.degrees(steering_angle)) <= 20:
            speed = 0.3
        else:
            speed = 0.2

        # Create and publish the AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.steering_angle_velocity = 8
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)