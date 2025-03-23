/**
 * @file: sub_node.cpp
 * @brief: 订阅/turtle1/pose话题并打印接收到的位姿信息
 * @author: WangZ
 * @date: 2025年3月
 * 
 * 详细描述：
 * 本文件包含一个ROS节点，用于订阅/turtle1/pose话题，
 * 并在接收到位姿信息时调用回调函数进行处理。
 * 
 * key:回调函数用 const turtlesim::Pose::ConstPtr& msg 处理
 *  const表明不会被回调函数修改，ConstPtr是智能指针，指向常量，非标准库
 */

#include <ros/ros.h>
#include <turtlesim/Pose.h>

// 回调函数，用于处理接收到的位姿信息
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("Turtle Pose: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "turtle_pose_subscriber");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅/turtle1/pose话题
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}
