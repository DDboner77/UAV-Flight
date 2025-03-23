#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "turtle_controller");
    ros::NodeHandle nh;

    // 创建发布者，发布到 /turtle1/cmd_vel 话题
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Rate loop_rate(10);  // 控制循环频率为 10Hz

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 2.0;   // 线速度（前进速度）
    vel_msg.angular.z = 1.8;  // 角速度（绕 Z 轴旋转速度）

    ROS_INFO("Start drawing a circle...");

    while (ros::ok()) {
        vel_pub.publish(vel_msg);  // 持续发布速度指令
        ROS_INFO("vel_msg.linear.x = %f, vel_msg.angular.z = %f", vel_msg.linear.x, vel_msg.angular.z);
        loop_rate.sleep();         // 保持循环频率
    }
    return 0;
}