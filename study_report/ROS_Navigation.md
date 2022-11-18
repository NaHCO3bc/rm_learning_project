# ROS Navigation

ros wiki：http://wiki.ros.org/navigation

## overview

从里程计和传感器流中获取信息，并输出速度命令以发送到移动基地。

运行ROS时，有一个tf tree，发布传感器数据

![attachment:overview_tf.png](http://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf_small.png)

## 硬件需求

* 仅适用于差速驱动和完整的轮式机器人
* 在移动底座安装平面激光器，该激光器用于地图构建和定位
* 接近方形或圆形的机器人上开发的性能最佳

## ROS Navigation Class

https://www.youtube.com/playlist?list=PLiiw0aSVHcAkF26qR6Q7x6RlLAL6-vuF3

### Basic concepts

* 首先需要一个地图
* 在地图中定位机器人所属位置，让机器人获得自己的位置信息
* 构建URDF：在urdf中需要有一个激光扫描的link

### Mapping&Create a map from zero

工具：rviz-显示laserscan和map

#### SLAM映射

**构建未知环境地图，同时在正在构建的地图中定位机器人的名称**：

​	 用gmapping包解决：ros wiki[http://wiki.ros.org/gmapping]

**slam_gmapping节点：构建一个2D地图**

* 使用来自激光和机器人位置的数据来创建二维地图
  * 从激光和机器人的tf中读取数据，并将这些数据转换为OGM（占用网络地图occupancy grid map）
    * 当map类型为OGM时：**/map** topic使用**/nav_msgs/OccupancyGrid**类型的数据：【0-100】表示障碍物数量，0表示无障碍，100表示完全为障碍，-1表示为未知

创建完地图后保存地图的描述：rosrun map_server map_saver -f map_name

* 生成.bgm和.yaml文件

 构建合适地图的硬件需求：

* 提供良好的激光数据和里程计数据
* general parameters
  * **base_frame(default:"base_link")：**表明附在移动底座上的帧名称
  * **map_frame(default:"map")：**表明附在map上的帧名称
  * **odom_frame(default:"odom")：**表明附在里程计系统上的帧名称
  * **map_update_interval(default:5.0)：**设置等待map更新的时间(seconds)

#### Transform

为了更好地使用激光数据，我们需要让系统知道激光在机器人中的位置，具体指向机器人中的一个点

* base_laser--base_link
* base_link--odom

### Robot Localization

拥有map之后，机器人需要知道每个时刻道路的位置和方向

rviz--PoseArray

#### Monte Carlo Localization(MCL 蒙特卡洛定位)

*粒子滤波器定位*

用于解决机器人技术中的定位问题：

* 一开始创建随机姿势，rviz中PoseArray的箭头十分分散且随机，与地图和环境有关

* 随着机器人开始运动，一开始猜测产生的随机的箭头会开始被丢弃
* 该算法由amcl包提供：ros wiki[http://wiki.ros.org/amcl]

#### amcl node

加载了amcl节点后

* 设置一个机器人的初始估计位置，帮助定位（即使一开始的定位并不准确）；通过**/initialpose** topic发布
* 开始移动机器人，amcl节点开始从**/scan**、**/map**、**/tf** topic获取激光、地图、转换数据；并将估计的机器人位置通过**/amcl_pose**、**/particlecloud** （通过rviz-PoseArray可视化）topic发布
* 硬件需求：好的激光数据、里程计数据和地图数据
* general parameters
  * **odom_model_type(default:"diff")：**用于里程计模型，可以为diff*(差分驱动)*、omni、diff-corrected、omni-corrected
  * **odom_frame_id(default:"odom")：**指明与里程计相关的坐标
  * **base_framne_id(default:"base_link")：**指明与机器人基座相关的坐标
  * **global_frame_id(default""map")：**表明定位系统发布的坐标系名称
  * **use_map_topic(default:"false")：**表明节点是否从topic或service获取到map数据

#### Transforms

需要一个laser_link-odom的tf 

amcl节点寻找base_link-base_laser的固定tf，因为amcl节点无法处理相对于底座移动的激光

### Path Planning

rviz：可视化路经规划（Path工具）

* 可视化地图，路径；2D工具（2D Pose Estimate，2D Nav Goal）

#### move_base node

主要实现移动机器人从当前位置到目标位置

* 执行*SimpleActionServer*，简单动作服务器可以获取数据类型为*geometry_msgs/PoseStamped*de的目标姿态；我们需要用*SimpleActionClient*给这个节点发送目标位置
* Action Server提供了**move_base/goal** topic，动作服务器是作为Navigation Stack的输入；该topic可以用于提供目标姿态

#### The Global Planner(全局规划器)

不同类型的全局规划器：Navfn，Corrot Planner，

* 当move_base node收到目标时，会将目标位置发送给Global Planner；Global Planner会计算一条安全路径以到达该目标的姿态

* 由于该路径是在机器人开始移动前计算好的，所以它不会考虑读取机器人在移动时的传感器数据

**适用于静态地图，利用global costmap计算路径**

move_base node提供了一个/make_plan的服务：只规划路径，不执行

#### Costmap

* Costmap是代表机器人在单元格网络中可以安全到达的位置的地图
* Costmap创建单元格并将地图划分为单元格，每个单元格都有一个二进制值：
  * 255 (NO_INFORMATION 没有信息)：保留没有足够已知信息的单元格
  * 254 (LETHAL_OBSTACLE 致命障碍)：表明感应到一个碰撞警告障碍在单元格内
  * 253 (INSCRIBED_INFLATED_OBSTACLE 内嵌充气障碍)：表明单元格内没有障碍，但是移动机器人中心到该单元格会引起碰撞（机器人边界维度问题）
  * 0 (FREE_SPACE 自由空间)：没有障碍的单元格，且移动机器人中心也不会引起碰撞
* 两种类型：global costmap，local costmap
  * 区别：构建方式： global costmap从一个静态地图构建，local costmap根据机器人传感器数据构建地图

* Global Costmap Parameters
  * **global_frame(default:"/map")：**costmap在其中运行的全局帧
  * **static_map(default: true)：**是否使用了静态地图来初始化costmap
  * **rolling_window(default: false)：**是否适用costmap的滚动窗口版本。若static_map为true，则该参数必须设置为false
  * **plugins：**插件

#### The Local Planner(本地规划器)

* global planner接受到目标位置后，基于静态地图计算得到一条全局路径，并将该路径发给local planner，然后local planner将执行全局规划的每个部分（local planner为global planner的一小部分）

* 由global planner给定一个规划和地图，然后loal planner会把这个规划拆分为多个小部分来执行，并提供一个速度命令来移动机器人
* local planner会监视里程计和激光数据，并选择global plan中的一个无碰撞的小部分来执行
* local planner可以在运动中重新计算机器人的路径，来避免碰撞
* local planner会将计算的路径会发布在**/local_plan** topic，并将尝试执行的小部分global plan发布在**/global_plan** topic

##### base_local_planner

* 从机器人的控制空间离散采样
* 对于每个采样速度，从机器人当前状态执行正向模拟，以预测如果应用采样速度会发生什么：一切正常/发生碰撞
*  评估正向模拟产生每条轨迹
* 丢弃非法轨迹（发生碰撞）
* 选择得分高的轨迹并将相关速度发送到移动基站
* 冲洗并重复

##### 其他local planner

dwa_local_planner,eband_local_planner,teb_local_planner

##### xy_goal_tolerance

坐标目标公差：给的值过高会使得机器人距目标位置较远时被认定为到达

#### Local Costmap

* local planner使用local costmap来计算local plans
* local costmap通过读取传感器数据来创建地图

##### Local Costmap Layers

* costmap_2d::ObstacleLayer：用于避障
* costmap_2d::InflationLayer：用于充气障碍

#### Recovery Behaviors

发生错误（如碰撞后无法移动）时会激活恢复行为，以尝试自行解决问题

##### Rotate Recovery

在z轴上旋转以查看是否可以找到离开的方法

#### Clear Costmap

* 清除local costmap中的所有障碍物，从0开始检测

* move_base node提供了 /move_base/clear_costmaps service来清除costmap的障碍
  * rosservice call /move_base/clear_costmaps "{}"

#### Oscillation Supression(抑制振荡)

可以修改一些参数以避免机器人在到达墙壁时保持摆动

## Setup Robot

### tf设置

1.为规避障碍物，需要用一个tf将base_link和base_laser联系起来：base_laser将障碍物的相对位置信息传给base_link从而实现避障。

2.依赖：roscpp，tf，geometry_msgs

#### tf广播变换

```c++
 1 #include <ros/ros.h>
   2 #include <tf/transform_broadcaster.h>
   3 
   4 int main(int argc, char** argv){
   5   ros::init(argc, argv, "robot_tf_publisher");
   6   ros::NodeHandle n;
   7 
   8   ros::Rate r(100);
   9 
  10   tf::TransformBroadcaster broadcaster;
  11 
  12   while(n.ok()){
  13     broadcaster.sendTransform(
  14       tf::StampedTransform(
  15         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
  16         ros::Time::now(),"base_link", "base_laser"));
  17     r.sleep();
  18   }
  19 }
```

总代码

```c++
   1 #include <ros/ros.h>
   2 #include <tf/transform_broadcaster.h>
```

包含tf/transform_broadcaster.h，简化发布tf任务

```c++
  10   tf :: TransformBroadcaster broadcaster；
```

创建对象，用来发送tf

```c++
  13     broadcaster.sendTransform(
  14       tf::StampedTransform(
  15         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
  16         ros::Time::now(),"base_link", "base_laser"));

```

使用TransformBroadcaster发送转换需要的五个参数。

* tf::Quaternion：用于两个坐标轴之间转换关系的rpy坐标旋转变换
* tf::Vector3：用于两个坐标轴之间转换关系的xyz的坐标平移变换
* 发布tf的时间戳
* 父link和子link

#### 使用tf变换

上面的部分创建了一个节点发布两个base_laser和base_link之间的转换关系

**现在需要一个节点来使用这个变换关系来获取base_laser帧中的一个点并将其转换为base_link帧中的一个点**

```c++
   1 #include <ros/ros.h>
   2 #include <geometry_msgs/PointStamped.h>
   3 #include <tf/transform_listener.h>
   4 
   5 void transformPoint(const tf::TransformListener& listener){
   6 //我们将在 laser_link 帧中创建一个点，我们希望将其转换为 base_link 帧
   7   geometry_msgs::PointStamped laser_point;
   8   laser_point.header.frame_id = "base_laser";
   9 
  10   //我们将只使用可用于简单示例的最新转换
  11   laser_point.header.stamp = ros::Time();
  12 
  13   //只是空间中的任意点
  14   laser_point.point.x = 1.0;
  15   laser_point.point.y = 0.2;
  16   laser_point.point.z = 0.0;
  17 
  18   try{
  19     geometry_msgs::PointStamped base_point;
  20     listener.transformPoint("base_link", laser_point, base_point);
  21 
  22     ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
  23         laser_point.point.x, laser_point.point.y, laser_point.point.z,
  24         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  25   }
  26   catch(tf::TransformException& ex){
  27     ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  28   }
  29 }
  30 
  31 int main(int argc, char** argv){
  32   ros::init(argc, argv, "robot_tf_listener");
  33   ros::NodeHandle n;
  34 
  35   tf::TransformListener listener(ros::Duration(10));
  36 
  37   //我们将每秒变换一个点
  38   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
  39 
  40   ros::spin();
  41 
  42 }
```

**☝总代码**

```c++
 3 #include <tf/transform_listener.h>
```

包括该头文件：TransformListerner对象通过ROS自动订阅转换消息主题并管理通过网络传入的所有转换数据

```c++
void transformPoint(const tf::TransformListener& listener){
```

创建一个transformPoint函数，给定TransformListener的形参，获取base_laser帧中的一个点并将其转换为base_link帧

**transformPoint函数作为main()函数中创建的ros::Timer的回调，每秒触发一次**

```c++
   6 //我们将在 laser_link 帧中创建一个点，我们希望将其转换为 base_link 帧
   7   geometry_msgs::PointStamped laser_point;
   8   laser_point.header.frame_id = "base_laser";
   9 
  10   //我们将只使用可用于简单示例的最新转换
  11   laser_point.header.stamp = ros::Time();
  12 
  13   //只是空间中的任意点
  14   laser_point.point.x = 1.0;
  15   laser_point.point.y = 0.2;
  16   laser_point.point.z = 0.0;
```

* 将创建的点作为geometry_msgs::PointStamped
  * 末尾的“Stamped”意味着它只包含一个标头，允许将时间戳和frame_id与消息相关联。
* 将laser_point消息的标志字段设置为ros::Time()，允许我们向TransformListener询问最新的可用转换
* frame_id设置为base_laser，并在base_laser中创建一个点

```c++
  18   try{
  19     geometry_msgs::PointStamped base_point;
  20     listener.transformPoint("base_link", laser_point, base_point);
  21 
  22     ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
  23         laser_point.point.x, laser_point.point.y, laser_point.point.z,
  24         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  25   }
```

有了“base_laser”框架中的点，想要将其转换为“base_link”框架。将使用TransformListener对象，并使用三个参数调用transformPoint()：

* 将点转换到的帧的名称(这里为base_link)
* 我们正在转换的点，以及存储转换点。

在调用transformPoint()之后，base_point 保存的信息与之前的 laser_point 相同，只是现在在“base_link”帧中。
