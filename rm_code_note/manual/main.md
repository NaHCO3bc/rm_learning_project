# main

## main.cpp

#include"manual_base.h，chassis_gimbal_shooter_cover_manual.h，engineer_manual.h"

```c++
int main(int argc, char **argv) {
  std::string robot;
  rm_manual::ManualBase *manual_control;
  ros::init(argc, argv, "rm_manual");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
```

**主函数：**定义字符串robot；定义manual_control

初始化ROS：定义节点名称：rm_manual

创造节点句柄nh，”~ ”命名空间：发布话题的名字为节点名字+话题名字

给字符串robot赋值：获取话题，“robot_type”参数，定义字符串“error”

```c++
if (robot == "standard")
    manual_control = new rm_manual::ChassisGimbalShooterCoverManual(nh);
  else if (robot == "hero")
    manual_control = new rm_manual::ChassisGimbalShooterManual(nh);
  else if (robot == "engineer")
    manual_control = new rm_manual::EngineerManual(nh);
  else {
    ROS_ERROR("no robot type ");
    return 0;
  }
```

如果为步兵机器人：manual_control赋值为构造函数ChassisGimbalShooterCoverManual；

如果为英雄机器人：manual_control赋值为构造函数ChassisGimbalShooterManual；

如果为工程机器人：manual_control赋值为构造函数EngineerManual；

否则报错：没有机器人类型

```c++
ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    manual_control->run();
    loop_rate.sleep();
  }
  return 0;
```

**指定循环频率为100Hz：**roscpp正常运行时：

spinOnce：确保回调函数可以被调用

manual_control运行run函数

sleep()：在剩余时间睡眠