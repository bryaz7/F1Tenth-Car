#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <cmath> 

constexpr double KP = 0.6;
constexpr double KD = 0.00;
constexpr double KI = 0.05;
constexpr double SERVO_OFFSET = 0.00;
constexpr int ANGLE_RANGE = 270;
constexpr double DESIRED_DISTANCE_RIGHT = 0.9;
constexpr double DESIRED_DISTANCE_LEFT = 0.55;
constexpr double CAR_LENGTH = 0.50;
constexpr double PI = 3.14159265358979323846;
constexpr double MAX_RANGE = 100.0;

class WallTracker {
public:
    WallTracker() : last_error(0.0), current_error(0.0), accumulated_error(0.0), current_velocity(0.0), time_difference(0.0) {
        command_publisher = node_handle.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        lidar_subscriber = node_handle.subscribe("/scan", 1000, &WallTracker::processLidarData, this);
        last_time = ros::Time::now().toSec();
    }

private:
    ros::NodeHandle node_handle;
    ros::Publisher command_publisher;
    ros::Subscriber lidar_subscriber;
    double last_error;
    double last_time;
    double current_error;
    double accumulated_error;
    double current_velocity;
    double time_difference;

    void controlUpdate() {
        ackermann_msgs::AckermannDriveStamped drive_command;
        double current_time = ros::Time::now().toSec();
        time_difference = current_time - last_time;
        accumulated_error += last_error * time_difference;
        drive_command.drive.steering_angle = -(KP * current_error + KD * (current_error - last_error) / time_difference + KI * accumulated_error);
        last_time = current_time;

        double absoluteSteeringAngle = abs(drive_command.drive.steering_angle);
        double speed;

        if (absoluteSteeringAngle > 20.0 / 180.0 * PI) {
            speed = 0.5;
        } else if (absoluteSteeringAngle > 10.0 / 180.0 * PI) {
            speed = 1.0;
        } else {
            speed = 1.5;
        }

        drive_command.drive.speed = speed;
        current_velocity = speed;
        command_publisher.publish(drive_command);
    }

    void processLidarData(const sensor_msgs::LaserScan& lidarData) {
        unsigned int index90 = static_cast<unsigned int>(floor((90.0 / 180.0 * PI - lidarData.angle_min) / lidarData.angle_increment));
        unsigned int index45;
        double angle45 = 45.0 / 180.0 * PI;

        if (lidarData.angle_min > angle45) {
            angle45 = lidarData.angle_min;
            index45 = 0;
        } else {
            index45 = static_cast<unsigned int>(floor((angle45 - lidarData.angle_min) / lidarData.angle_increment));
        }

        double range45 = std::isinf(lidarData.ranges[index45]) || std::isnan(lidarData.ranges[index45]) ? MAX_RANGE : lidarData.ranges[index45];
        double range90 = std::isinf(lidarData.ranges[index90]) || std::isnan(lidarData.ranges[index90]) ? MAX_RANGE : lidarData.ranges[index90];


        double alpha = atan((range45 * cos(90.0 / 180.0 * PI - angle45) - range90) / (range45 * sin(90.0 / 180.0 * PI - angle45)));
        double dt_1 = range90 * cos(alpha);
        double dt_2 = dt_1 + 1.00 * sin(alpha);
        current_error = DESIRED_DISTANCE_LEFT - dt_2;

        ROS_INFO_STREAM("current_error");
        ROS_INFO_STREAM(current_error);

        controlUpdate();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_tracker");
    WallTracker wallTracker;
    ros::spin();
    return 0;
}