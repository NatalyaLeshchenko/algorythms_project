#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    float min_dist = *std::min_element(scan->ranges.begin(), scan->ranges.end());
    float obstacle_threshold = 0.5; // Стоп, если препятствие ближе 0.5 м

    geometry_msgs::Twist cmd_vel;
    
    if (min_dist < obstacle_threshold) {
        cmd_vel.linear.x = 0.0;       // Остановка
        cmd_vel.angular.z = 0.5;      // Поворот
    } else {
        cmd_vel.linear.x = 0.2;       // Движение вперёд
        cmd_vel.angular.z = 0.0;
    }

    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "autonomous_nav");
    ros::NodeHandle nh;

    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, lidarCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();
    return 0;
}
