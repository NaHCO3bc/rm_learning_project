## chassis_gimbal_shooter_cover_manual

## chassis_gimbal_shooter_cover_manual.h

```c++
namespace rm_manual {
class ChassisGimbalShooterCoverManual : public ChassisGimbalShooterManual {
 public:
  explicit ChassisGimbalShooterCoverManual(ros::NodeHandle &nh);
  void run() override;
```

**继承ChassisGimbalShooterManual：**

public：

构造函数ChassisGimbalShooterCoverManual

定义函数run()

```c++
protected:
  void updatePc() override;//更新电脑端数据
  void checkKeyboard() override;//检查键盘
  void sendCommand(const ros::Time &time) override;//发送命令
  void gimbalOutputOn() override;//云台输出开启
  void remoteControlTurnOff() override;//远程控制关闭
  void remoteControlTurnOn() override;//远程控制开启
  void drawUi(const ros::Time &time) override;//画UI
  void rightSwitchDownRise() override;//右侧开关下方升起
  void rightSwitchMidRise() override;//右侧开关中间升起
  void rightSwitchUpRise() override;//右侧开关上方升起
  void ctrlZPress();//按压ctrlZ键
  void ctrlZRelease() { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); };//释放ctrlZ键：设置云台模式为速度RATE
  void ctrlQPress();//按压ctrlQ键
```

```c++
  rm_common::JointPositionBinaryCommandSender *cover_command_sender_{};
  rm_common::CalibrationQueue *cover_calibration_;
  InputEvent ctrl_z_event_, ctrl_q_event_;
};
```

**protected:**

构造函数；定义关节位置二进制命令发送者：覆盖命令发送者

定义校准队列：覆盖校准

定义输入事件：ctrlZ事件。ctrlQ键

## chassis_gimbal_shooter_cover_manual.cpp

```c++
namespace rm_manual {
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle &nh) : ChassisGimbalShooterManual(nh) {
  //创造cover_nh节点句柄，命名空间cover
  ros::NodeHandle cover_nh(nh, "cover");
  cover_command_sender_ = new rm_common::JointPositionBinaryCommandSender(cover_nh);//覆盖命令发布者构造函数：关节位置二进制命令发送者
  XmlRpc::XmlRpcValue rpc_value;//远程位置控制有效
  nh.getParam("cover_calibration", rpc_value);////获取远程位置控制有效参数，命名空间为覆盖校准器
  cover_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);//覆盖校准器构造函数：校准器队列
  ctrl_z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
                        boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));//ctrlz事件设置边界：按压/释放crtlz
  ctrl_q_event_.setRising(boost::bind(&ChassisGimbalShooterCoverManual::ctrlQPress, this));//ctrlq事件设置升起：按压ctrlq
}
```

```c++
void ChassisGimbalShooterCoverManual::run() {
  ChassisGimbalShooterManual::run();
  cover_calibration_->update(ros::Time::now());
}
```

**运行函数：**覆盖校准器->更新实时时间

```c++
void ChassisGimbalShooterCoverManual::updatePc() {
  ChassisGimbalShooterManual::updatePc();
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x * gimbal_scale_,
                              cover_command_sender_->getState() ? 0.0 : data_.dbus_data_.m_y * gimbal_scale_);
}
```

**更新电脑端数据：**设置云台速率：-鼠标x轴数据 * 云台位移改变量，覆盖命令发布者获取模式 ? 0 : 鼠标y轴数据 * 云台位移改变量

```c++
void ChassisGimbalShooterCoverManual::checkKeyboard() {
  ChassisGimbalShooterManual::checkKeyboard();
  ctrl_z_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
  ctrl_q_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_q);
}
```

**检查键盘：**ctrlz事件更新数据：ctrl键数据 & z键数据；ctrlq事件更新数据：ctrl键数据 & q键数据

```c++
void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time &time) {
  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
}
```

**发送命令：**覆盖命令发布者发布命令

```c++
void ChassisGimbalShooterCoverManual::gimbalOutputOn() {
  ChassisGimbalShooterManual::gimbalOutputOn();
  cover_calibration_->reset();
}
```

**云台输出开启：**覆盖校准器重新设置->如果校准器通信不为空，return；校准器通信开始，开关为false；查询通讯，，应答已经校准

```c++
void ChassisGimbalShooterCoverManual::remoteControlTurnOff() {
  ChassisGimbalShooterManual::remoteControlTurnOff();
  cover_calibration_->stop();
}
```

**远程控制关闭：**覆盖校准器->停止校准

```c++
void ChassisGimbalShooterCoverManual::remoteControlTurnOn() {
  ChassisGimbalShooterManual::remoteControlTurnOn();
  cover_calibration_->stopController();
}
```

**远程控制开启：**覆盖校准器：关闭控制器->如果校准器通信为空则退出；如果校准器通信!=最终值 && 开关是否打开，则controller_manager关闭控制器

```c++
void ChassisGimbalShooterCoverManual::drawUi(const ros::Time &time) {
  ChassisGimbalShooterManual::drawUi(time);
  flash_ui_->update("cover", time, !cover_command_sender_->getState());
}
```

**画UI：**光灯UI->更新数据：cover命名空间，实时时间，!覆盖命令发送者模式

```c++
void ChassisGimbalShooterCoverManual::rightSwitchDownRise() {
  ChassisGimbalShooterManual::rightSwitchDownRise();
  cover_command_sender_->on();
}
```

**右侧开关下方升起：**覆盖命令发布者：获取开启位置，state状态为true

```c++
void ChassisGimbalShooterCoverManual::rightSwitchMidRise() {
  ChassisGimbalShooterManual::rightSwitchMidRise();
  cover_command_sender_->off();
}
```

**右侧开关中间升起：**覆盖命令发布者：获取关闭位置，state状态为false

```c++
void ChassisGimbalShooterCoverManual::rightSwitchUpRise() {
  ChassisGimbalShooterManual::rightSwitchUpRise();
  cover_command_sender_->off();
}
```

**右侧开关上方升起：**覆盖命令发布者：获取关闭位置，state状态为false

```c++
void ChassisGimbalShooterCoverManual::ctrlZPress() {
  geometry_msgs::PointStamped aim_point{};
  aim_point.header.frame_id = "yaw";
  aim_point.header.stamp = ros::Time(0);
  aim_point.point.x = 1;
  aim_point.point.y = 0;
  aim_point.point.z = 0;
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  cover_command_sender_->getState() ? cover_command_sender_->off() : cover_command_sender_->on();
}
```

**按压ctrlZ键：**

获取目标点参考坐标系和时间戳

获取目标点坐标系id：yaw

获取执行器测量时间

设置目标点坐标为(1,0,0)

设置云台模式为检测DIRECT

设置底盘模式为跟随FOLLOW

更新底盘功率限制模式为正常NORMAL

获取覆盖命令发送者模式 ? 获取关闭位置，state状态为false : 获取开启位置，state状态为true

```c++
void ChassisGimbalShooterCoverManual::ctrlQPress() {
  cover_calibration_->reset();
}
```

**按压ctrlQ键：**覆盖校准器重新设置->如果校准器通信不为空，return；校准器通信开始，开关为false；查询通讯，，应答已经校准