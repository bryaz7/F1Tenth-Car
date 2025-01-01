#include "math.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


class Safety {
public:
    Safety() {
        sub_scan = nh.subscribe("/scan", 1000, &Safety::scan_callback, this);
        sub_drive = nh.subscribe("/odom", 1000, &Safety::drive_callback, this);
        pub_1 = nh.advertise<std_msgs::Bool>("/brake_bool", 1000);
        pub_2 = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);

    }

    void publish() {
        double vx = drive_info.twist.twist.linear.x;
        double vy = drive_info.twist.twist.linear.y;
        double TTC_threshold = 0.4;
        double min_TTC = 100;
        
        for (int i = 0; i < scan_info.ranges.size(); i++) {
            if (!std::isinf(scan_info.ranges[i]) && !std::isnan(scan_info.ranges[i])) {
                double alpha = scan_info.angle_min + scan_info.angle_increment * i;
                double dr = cos(alpha) * vx + sin(alpha) * vy;
                double r = scan_info.ranges[i];
                if (dr > 0 && r / dr < min_TTC) min_TTC = r / dr;
            }
        }
        if (min_TTC <= TTC_threshold) {
            std_msgs::Bool brake_bool_result;
            brake_bool_result.data = true;
            pub_1.publish(brake_bool_result);
            ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
            ackermann_drive_result.drive.speed = 0.0;
            pub_2.publish(ackermann_drive_result);
        } else {
            std_msgs::Bool brake_bool_result;
            brake_bool_result.data = false;
            pub_1.publish(brake_bool_result);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_drive;
    sensor_msgs::LaserScan scan_info;
    nav_msgs::Odometry drive_info;

    void scan_callback(const sensor_msgs::LaserScan& lidar_info) {
        scan_info = lidar_info;
    }

    void drive_callback(const nav_msgs::Odometry& odometry_info) {
        drive_info = odometry_info;
    }

}; 

int main(int argc, char** argv) {

    ros::init(argc, argv, "safety_node");
    Safety safety;
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        safety.publish();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}