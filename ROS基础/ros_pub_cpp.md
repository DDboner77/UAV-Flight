# ROS编程实现发布话题控制小海龟

## 1. 在已有工作空间中创建功能包

```bash
# 创建功能包
cd src
catkin_create_pkg my_turtle_control roscpp geometry_msgs
```

格式：`catkin_create_pkg` 功能包名称 依赖项
`roscpp`：C++标准库
`geometry_msgs`：速度消息类型库

## 2. 在src目录下创建发布节点

`在创建包下的src创建代码文件`

```bash
cd ~/catkin_ws/src/my_turtle_control/src
touch pub_node.cpp           # 创建发布节点文件
```

## 3. 编写发布节点代码

**打开pub_node.cpp文件，编写发布节点代码**
```c++
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char ​**argv) {
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
```
## 4.更改CMakeLists.txt文件


在创建包下的与`src`同级的`CMakeLists.txt`文件中添加以下内容：

```cmake
add_executable(pub_node src/pub_node.cpp)
target_link_libraries(pub_node ${catkin_LIBRARIES})
```

## 5. 编译

```bash
cd ~/catkin_ws
catkin_make
```

## 6. 运行

```bash
source devel/setup.bash  # 激活工作空间,必要步骤
rosrun my_turtle_control pub_node
```

## 7. 观察效果

在终端中运行 `rosrun turtlesim turtlesim_node` 启动小乌龟仿真环境，然后运行 `rosrun my_turtle_control pub_node` 启动发布节点，可以看到小乌龟在画圆。
