# ROS基础指令学习

## 一、工作空间和功能包指令

### 1.创建工作空间

```bash
mkdir ~/catkin_ws/src
// 在src工作路径下初始化工作空间
catkin_init_workspace
```

### 2.编译工作空间

```bash
// 在工作空间路径下编译
cd ~/catkin_ws
catkin_make
// 编译完成后设置环境变量   
source devel/setup.bash
```

### 3.创建功能包

**package_name**: 功能包名称

**std_msgs**: 标准消息功能包

**rospy**: python编程接口

__roscpp__: C++编程接口

以上为一般基础功能包，一般创建时都需要添加

```bash
cd ~/catkin_ws/src
// 创建功能包，需要指定依赖的功能包 
catkin_create_pkg package_name std_msgs rospy roscpp
```

### 4.编译功能包

```bash
// 添加新的功能包以后，需要重新编译工作空间
cd ~/catkin_ws
catkin_make
```

## 二、节点指令

### 1.启动ROS主核心

```bash
// 使用ros的前提是启动ros主核心
roscore
```

### 2.创建节点

**使用自己的功能包创建节点，需要先在src下存放代码并添加CMakeLists.txt文件，生成可执行文件**

```CMake
add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})  # 链接所有依赖库
```

```bash
rosrun package_name executable_file
```

### 3.查看节点

```bash
rosnode list
```

### 4.查看节点信息

```bash
rosnode info node_name
```

### 5.关闭节点

```bash
rosnode kill node_name
```
或者在当前终端 crtl+c 退出

## 三、话题指令

### 1.发布话题

```bash
rostopic pub topic_name std_msgs/String "hello"
```

### 2.查看话题

```bash
```bash
rosrun package_name executable_file
```

### 3.查看节点

```bash
rosnode list
```

### 4.查看节点信息

```bash
rosnode info node_name
```

### 5.关闭节点

```bash
rosnode kill node_name
```
或者在当前终端 crtl+c 退出
