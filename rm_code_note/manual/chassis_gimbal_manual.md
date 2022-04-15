# chassis_gimbal_manual

## chassis_gimbal_manual.h

```c++
class ChassisGimbalManual : public ManualBase {
 public:
  explicit ChassisGimbalManual(ros::NodeHandle &nh);
 protected:
  void sendCommand(const ros::Time &time) override;
  void updateRc() override;
  void updatePc() override;
  void checkReferee() override;
  void checkKeyboard() override;
  void drawUi(const ros::Time &time) override;
  void remoteControlTurnOff() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchMidFall() override;
  void leftSwitchDownRise() override;
```

**继承ManualBase类：**（override：确保该函数为虚函数并覆写来自基类的虚函数，避免继承过程中函数名出现错误而没有报错）

public：构造ChassisGimbalManual函数

protected：继承函数：发送命令，更新遥控器数据，更新电脑端数据，检查裁判系统，检查键盘，画UI，关闭远程连接，右侧开关下侧上升，右开关中间上升，右开关上侧上升，左开关中间下降，左开关下侧上升

```c++
  virtual void wPress() { x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0; }
  virtual void wRelease();
  virtual void sPress() { x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0; }
  virtual void sRelease();
  virtual void aPress() { y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0; }
  virtual void aRelease();
  virtual void dPress() { y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0; }
  virtual void dRelease();
```

**定义多态函数：**

按压w键：x方向的位移改变量 = 若位移量>=1，则位移改变量为1，否则为位移改变量+1

按压s键：x方向的位移改变量 = 若位移量<=-1，则位移改变量为-1，否则为位移改变量-1

按压a键：y方向的位移改变量 = 若位移量>=1，则位移改变量为1，否则为位移改变量+1

按压d键：y方向的位移改变量 = 若位移量<=-1，则位移改变量为-1，否则为位移改变量-1

w/s/a/d键释放（release）

```c++
  void wPressing();
  void aPressing();
  void sPressing();
  void dPressing();
  void mouseMidRise();
```

**定义w/a/s/d按压中函数，鼠标中间上升函数**

```c++
 rm_common::ChassisCommandSender *chassis_cmd_sender_{};
  rm_common::Vel2DCommandSender *vel_cmd_sender_{};
  rm_common::GimbalCommandSender *gimbal_cmd_sender_{};
  TimeChangeUi *time_change_ui_{};
  FlashUi *flash_ui_{};
  TriggerChangeUi *trigger_change_ui_{};
  FixedUi *fixed_ui_{};
  double x_scale_{}, y_scale_{};
  double gimbal_scale_{1.};
  double gyro_move_reduction_{1.};
```

**定义成员：**

定义指针：底盘命令发送者，速度命令发送者，云台命令发送者

​                   UI：改变时间，装甲板闪光灯，扳机改变，锁定

定义数组：x轴位移改变量，y轴位移改变量，云台位移改变量，陀螺仪移动减少量

```c++
InputEvent chassis_power_on_event_, gimbal_power_on_event_, w_event_, s_event_, a_event_, d_event_, mouse_mid_event_;
```

定义上传事件：底盘电源开启，云台电源开启，w/s/a/d键，鼠标中间

## chassis_gimbal_manual.cpp

```c++
ChassisGimbalManual::ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh) {
  //创造chassis_nh节点句柄,chassis命名空间
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_.referee_.referee_data_);
    
  //创造vel_nh节点句柄，vel命名空间  
  ros::NodeHandle vel_nh(nh, "vel");
  vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
    ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
    
  //创造gimbal_nh节点句柄,命名空间  
  ros::NodeHandle gimbal_nh(nh, "gimbal");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_.referee_.referee_data_);
    
  //创造ui_nh节点句柄,ui命名空间  
  ros::NodeHandle ui_nh(nh, "ui");
  trigger_change_ui_ = new TriggerChangeUi(ui_nh, data_);
  time_change_ui_ = new TimeChangeUi(ui_nh, data_);
  flash_ui_ = new FlashUi(ui_nh, data_);
  fixed_ui_ = new FixedUi(ui_nh, data_);
```

**new关键词构造函数（在堆上分配空间，并在该空间内调用对象的构造函数）：**

vel_nh：若陀螺仪陀螺仪减少量为0，则报错：陀螺仪减少量没有界定，显示此时的命名空间

```c++
chassis_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::chassisOutputOn, this));
  gimbal_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::gimbalOutputOn, this));
  w_event_.setEdge(boost::bind(&ChassisGimbalManual::wPress, this),
                   boost::bind(&ChassisGimbalManual::wRelease, this));
  w_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::wPressing, this));
  s_event_.setEdge(boost::bind(&ChassisGimbalManual::sPress, this),
                   boost::bind(&ChassisGimbalManual::sRelease, this));
  s_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::sPressing, this));
  a_event_.setEdge(boost::bind(&ChassisGimbalManual::aPress, this),
                   boost::bind(&ChassisGimbalManual::aRelease, this));
  a_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::aPressing, this));
  d_event_.setEdge(boost::bind(&ChassisGimbalManual::dPress, this),
                   boost::bind(&ChassisGimbalManual::dRelease, this));
  d_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::dPressing, this));
  mouse_mid_event_.setRising(boost::bind(&ChassisGimbalManual::mouseMidRise, this));
}
```

**构造函数：**

底盘/云台电源打开->设置上升：底盘/云台输出打开

w键：设置边界（按，释放）；设置高电平（持续按压）；

s键：设置边界（按，释放）；设置高电平（持续按压）；

a键：设置边界（按，释放）；设置高电平（持续按压）；

d键：设置边界（按，释放）；设置高电平（持续按压）；

鼠标中间：设置升起

```c++
void ChassisGimbalManual::sendCommand(const ros::Time &time) {
  chassis_cmd_sender_->sendCommand(time);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}
```

**设置命令：**底盘/速度/云台命令发送者

```c++
void ChassisGimbalManual::updateRc() {
  ManualBase::updateRc();
  if (std::abs(data_.dbus_data_.wheel) > 0.01) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  } else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setAngularZVel(data_.dbus_data_.wheel);
  vel_cmd_sender_->setLinearXVel(data_.dbus_data_.ch_r_y);
  vel_cmd_sender_->setLinearYVel(-data_.dbus_data_.ch_r_x);
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
}

```

**更新遥控模式数据：**

如果dbus中控制小陀螺的波浪轮数据的绝对值>0.01，则设置底盘模式为GYPO小陀螺

否则设置底盘模式为FOLLOW跟随，设置z轴角速度为控制小陀螺的波浪轮数值，x轴线速度为右操作杆y方向的操作速度，y轴线速度为右操作杆x方向的操作速度

设置云台的速度：x轴速度为左操作杆x方向的操作速度的负数，y轴线速度为左操作杆y方向的操作速度

```c++
void ChassisGimbalManual::updatePc() {
  ManualBase::updatePc();
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x * gimbal_scale_, data_.dbus_data_.m_y * gimbal_scale_);
}
```

**更新电脑端数据：**设置云台速率：x轴速度为左操作杆x方向的操作速度的负数 * 云台的位移改变量，y轴线速度为左操作杆y方向的操作速度 *  云台的位移改变量

```c++
void ChassisGimbalManual::checkReferee() {
  ManualBase::checkReferee();
  chassis_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_);
  gimbal_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_);
}
```

**检查裁判系统：**更新比赛机器人状态下主电源底盘/云台输出：1

```c++
void ChassisGimbalManual::checkKeyboard() {
  ManualBase::checkKeyboard();
  if (data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_ENGINEER
      || data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_ENGINEER) {
    w_event_.update((!data_.dbus_data_.key_ctrl) & (!data_.dbus_data_.key_shift) & data_.dbus_data_.key_w);
    s_event_.update((!data_.dbus_data_.key_ctrl) & (!data_.dbus_data_.key_shift) & data_.dbus_data_.key_s);
    a_event_.update((!data_.dbus_data_.key_ctrl) & (!data_.dbus_data_.key_shift) & data_.dbus_data_.key_a);
    d_event_.update((!data_.dbus_data_.key_ctrl) & (!data_.dbus_data_.key_shift) & data_.dbus_data_.key_d);
  } else {
    w_event_.update(data_.dbus_data_.key_w);
    s_event_.update(data_.dbus_data_.key_s);
    a_event_.update(data_.dbus_data_.key_a);
    d_event_.update(data_.dbus_data_.key_d);
  }
  mouse_mid_event_.update(data_.dbus_data_.m_z != 0.);
}
```

**检查键盘：**

如果裁判系统中机器人id为红方工程机器人 || id为蓝方工程机器人，则更新w/s/a/d键-> !ctrl键 & !shift键&w/s/a/d键

否则更新w/s/a/d键 -> w/s/a/d键

更新鼠标中间 ->鼠标z键位 !=0

```c++
void ChassisGimbalManual::drawUi(const ros::Time &time) {
  ManualBase::drawUi(time);
  time_change_ui_->update("capacitor", time);
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID && data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::TEST);
    trigger_change_ui_->update("chassis", chassis_cmd_sender_->getMsg()->mode, false, 1, false);
  } else {
    trigger_change_ui_->update("chassis", chassis_cmd_sender_->getMsg()->mode,
                               chassis_cmd_sender_->power_limit_->getState() == rm_common::PowerLimit::BURST, 0,
                               chassis_cmd_sender_->power_limit_->getState() == rm_common::PowerLimit::CHARGE);
  }
  flash_ui_->update("spin", time, chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO
      && vel_cmd_sender_->getMsg()->angular.z != 0.);
  flash_ui_->update("armor0", time);
  flash_ui_->update("armor1", time);
  flash_ui_->update("armor2", time);
  flash_ui_->update("armor3", time);
}
```

**画UI：**

***更新UI改变时间（电容）***

如果dbus左叶轮数据等于遥控器中间位 && dbus右叶轮数据等于遥控器上升位，则底盘命令发送者更新

功率限制，设置模式为测试（TEST=0）；

***UI扳机改变更新：***chassis命名空间，底盘命令发送者获取mode

否则UI扳机改变更新：chassis命名空间，底盘命令发送者获取mode，设置功率限制模式为突发（BURST=1），然后设置功率限制模式为充电（CHARGE=3）

***装甲板UI***

更新数据：旋转，底盘实时模式 == GYPO &&  z轴角速度 != 0

更新0/1/2/3号装甲执行器实时时间

```c++
void ChassisGimbalManual::remoteControlTurnOff() {
  ManualBase::remoteControlTurnOff();
  vel_cmd_sender_->setZero();
  chassis_cmd_sender_->setZero();
  gimbal_cmd_sender_->setZero();
}
```

**远程控制关闭：**controller_manager停止主控制器和校准控制器，模式为被动

设置x轴，y轴线速度为0，z轴角速度为0

设置底盘功率限制为实时功率限制

设置云台yaw轴，pitch轴速度为0

```c++
void ChassisGimbalManual::rightSwitchDownRise() {
  ManualBase::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_cmd_sender_->setZero();
}
```

**右侧开关下方升起：**模式为空闲

设置底盘模式为FOLLOW

设置x轴，y轴线速度为0，z轴角速度为0

设置云台模式为RATE

设置云台yaw轴，pitch轴速度为0

```c++
void ChassisGimbalManual::rightSwitchMidRise() {
  ManualBase::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}
```

**右侧开关中间升起：**模式为遥控

设置底盘模式为FOLLOW

设置云台模式为RATE

```c++
void ChassisGimbalManual::rightSwitchUpRise() {
  ManualBase::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  trigger_change_ui_->add();
  time_change_ui_->add();
  fixed_ui_->add();
}
```

**右侧开关上方升起：**模式为电脑控制

设置底盘模式为FOLLOW

设置x轴，y轴线速度为0，z轴角速度为0

设置云台模式为RATE

UI扳机改变，时间改变，锁定：在地图上矢量运行

```c++
void ChassisGimbalManual::leftSwitchMidFall() {
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}
```

**左侧开关中间下降：**设置底盘功率为充电CHARGE

```c++
void ChassisGimbalManual::leftSwitchDownRise() {
  ManualBase::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}
```

**左侧开关下方上升：**设置云台模式为速度RATE

```c++
void ChassisGimbalManual::wPressing() {
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
}
```

**持续按压w键：**设置x轴线速度（获取底盘模式 == GYPO ？x轴位移改变量 * 陀螺仪移动的减少量 ：x轴位移改变量）

```c++
void ChassisGimbalManual::wRelease() {
  x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  vel_cmd_sender_->setLinearXVel(x_scale_);
}
```

**释放w键：**x轴位移改变量= x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0

设置x轴线速度为x轴位移改变量

```c++
void ChassisGimbalManual::sPressing() {
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
}
```

**持续按压s键：**设置x轴线速度（获取底盘模式 == GYPO ？x轴位移改变量 * 陀螺仪移动的减少量 ：x轴位移改变量）

```c++
void ChassisGimbalManual::sRelease() {
  x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  vel_cmd_sender_->setLinearXVel(x_scale_);
}
```

**释放s键：**x轴位移改变量= x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0

设置x轴线速度为x轴位移改变量

```c++
void ChassisGimbalManual::aPressing() {
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
}
```

**持续按压a键：**设置y轴线速度（获取底盘模式 == GYPO ？y轴位移改变量 * 陀螺仪移动的减少量 ：y轴位移改变量）

```c++
void ChassisGimbalManual::aRelease() {
  y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  vel_cmd_sender_->setLinearYVel(y_scale_);
}
```

**释放a键：**y轴位移改变量= y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0

设置y轴的线速度为y轴的位移改变量

```c++
void ChassisGimbalManual::dPressing() {
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
}
```

**持续按压p轴：**设置y轴线速度（获取底盘模式 == GYPO ？y轴位移改变量 * 陀螺仪移动的减少量 ：y轴位移改变量）

```c++
void ChassisGimbalManual::dRelease() {
  y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  vel_cmd_sender_->setLinearYVel(y_scale_);
}
```

**释放d键：**y轴位移改变量= y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0

设置y轴的线速度为y轴的位移改变量

```c++
void ChassisGimbalManual::mouseMidRise() {
  if (gimbal_scale_ >= 0. && gimbal_scale_ <= 3.) {
    if (gimbal_scale_ + 0.2 <= 3. && data_.dbus_data_.m_z > 0.) gimbal_scale_ += 0.2;
    else if (gimbal_scale_ - 0.2 >= 0. && data_.dbus_data_.m_z < 0.) gimbal_scale_ -= 0.2;
  }
}
```

**鼠标中间上升：**

如果（云台位移改变量 >=0 && 云台位移改变量 <=3）{

如果（云台位移改变量 + 0.2 <=3 && 鼠标z轴 >0）云台位移改变量 += 0.2

否则（云台位移改变量 - 0.2 >= 0 && 鼠标z轴 <0）云台位移改变量 -= 0.2

}
