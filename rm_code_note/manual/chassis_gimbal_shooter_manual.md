# chassis_gimbal_shooter_manual

## chassis_gimbal_shooter_manual.h

```c++
namespace rm_manual {
class ChassisGimbalShooterManual : public ChassisGimbalManual {
 public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle &nh);
  void run() override;
```

**继承ChassisGimbalManual类：**

public:

设置多态函数ChassisGimbalShooterManual，设置run()函数

```c++
protected:
  void checkReferee() override;//检查裁判系统
  void checkKeyboard() override;//检查键盘
  void updateRc() override;//更新遥控器数据
  void updatePc() override;//更新电脑端数据
  void sendCommand(const ros::Time &time) override;//发送命令
  void chassisOutputOn() override;//底盘输出开启
  void shooterOutputOn() override;//发射输出开启
  void selfInspectionStart() { trigger_calibration_->reset(); };//开始自检：扳机校准器重置
  void gameStart() { trigger_calibration_->reset(); };//比赛开始：扳机校准器重置
  void remoteControlTurnOff() override;//远程控制关闭
  void remoteControlTurnOn() override;//远程控制开启
  void robotDie() override;//机器人死亡
  void drawUi(const ros::Time &time) override;//画UI
  void rightSwitchDownRise() override;//右侧开关下方升起
  void rightSwitchMidRise() override;//右侧开关中间升起
  void rightSwitchUpRise() override;//右侧开关上方升起
  void leftSwitchDownRise() override;//左侧开关下方升起
  void leftSwitchMidRise() override;//左侧开关中间升起
  void leftSwitchUpRise() override;//左侧开关上方升起
  void leftSwitchUpOn(ros::Duration duration);//左侧开关上方开启（持续时间）
  void mouseLeftPress();//按压鼠标左键
  void mouseLeftRelease() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY); }//释放鼠标左键（发射命令发送者->设置发射模式为准备READY）
  void mouseRightPress();//按压鼠标右键
  void mouseRightRelease() { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }//释放鼠标右键->（云台命令发送者->设置云台模式为速度RATE）
  void wPress() override;//按压w键
  void aPress() override;//按压a键
  void sPress() override;//按压s键
  void dPress() override;//按压d键
  void ePress();//按压e键
  void gPress();//按压g键
  void bPress();//按压b键
  void qPress() { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }//按压q键：{射击命令发送者->设置突发模式 [热量限制 ->（!发送命令发送者 ->（热量限制-> 突发标志））]}
  void fPress() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }//按压f键：射击命令发送者设置模式为STOP
  void shiftPress();//按压shift键
  void shiftRelease();//释放shift键
  void ctrlCPress();//按压ctrl+c键
  void ctrlVPress();//按压ctrl+v键
  void ctrlRPress();//按压ctrl+r键
  void ctrlBPress();//按压ctrl+b键
```

```c++
InputEvent shooter_power_on_event_, self_inspection_event_, game_start_event_, e_event_, g_event_, q_event_, f_event_,
      b_event_, ctrl_c_event_, ctrl_v_event_, ctrl_r_event_, ctrl_b_event_, shift_event_, mouse_left_event_,
      mouse_right_event_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::SwitchDetectionCaller *switch_detection_srv_{};
  rm_common::CalibrationQueue *trigger_calibration_;
};
```

**定义输入事件：**开启发射电源，自检测，比赛开始，e/g/q/f/b键，ctrl+c/v/r/b键，shift键，鼠标左右键，发射命令发送者，开关检测服务资源，扳机校准

## chassis_gimbal_shooter_manual.cpp

```c++
namespace rm_manual {
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
  //创造节点句柄shooter_nh，命名空间shooter  
  ros::NodeHandle shooter_nh(nh, "shooter");
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);//发射命令发布者构造函数
    
  //创造节点句柄个detection_switch_nh，命名空间detection_switch  
  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);//选择检测通信
  XmlRpc::XmlRpcValue rpc_value;//远程位置控制有效
  nh.getParam("trigger_calibration", rpc_value);//获取trigger_calibration数据，远程位置控制有效
  trigger_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);//扳机校准构造函数：校准队列
  shooter_power_on_event_.setRising(boost::bind(&ChassisGimbalShooterManual::shooterOutputOn, this));//发射电源开启事件构造函数：设置升起->发射输出开启
  self_inspection_event_.setRising(boost::bind(&ChassisGimbalShooterManual::selfInspectionStart, this));//自检时间构造函数：设置升起->自检开始
  game_start_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gameStart, this));//比赛开始事件构造函数：设置升起->比赛开始
  left_switch_up_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchUpOn, this, _1));//左开关上方事件：设置高电平->左开关上方开启
  e_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ePress, this));//e键构造函数：按压e键
  g_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gPress, this));//g键构造函数：按压g键
  q_event_.setRising(boost::bind(&ChassisGimbalShooterManual::qPress, this));//q键构造函数：按压q键
  f_event_.setRising(boost::bind(&ChassisGimbalShooterManual::fPress, this));//f键构造函数：按压f键
  b_event_.setRising(boost::bind(&ChassisGimbalShooterManual::bPress, this));//b键构造函数：按压b键
  ctrl_c_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlCPress, this));//ctrlc键构造函数：按压ctrlc键
  ctrl_v_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this));//ctrlv键构造函数：按压ctrlv键
  ctrl_r_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this));//ctrlr键构造函数：按压ctrlr键
  ctrl_b_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlBPress, this));//ctrlb键构造函数：按压ctrlb键
  shift_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::shiftPress, this),//shift键构造函数：设置边缘->按压shift键
                       boost::bind(&ChassisGimbalShooterManual::shiftRelease, this));//释放shift键
  mouse_left_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::mouseLeftPress, this));//鼠标左键事件构造函数：设置高电平->按压鼠标左键
  mouse_left_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::mouseLeftRelease, this));//鼠标左键事件构造函数：设置下降->释放鼠标左键
  mouse_right_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::mouseRightPress, this));//鼠标右键事件构造函数：设置高电平->按压鼠标右键
  mouse_right_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::mouseRightRelease, this));//鼠标右键事件构造函数：设置下降->释放鼠标右键
}

```

```c++
void ChassisGimbalShooterManual::run() {
  ChassisGimbalManual::run();
  switch_detection_srv_->setEnemyColor(data_.referee_.referee_data_);
  trigger_calibration_->update(ros::Time::now());
}
```

**运行函数：**选择检测通信->设置敌人机器人颜色；扳机校准->更新实时时间

```c++
void ChassisGimbalShooterManual::checkReferee() {
  ChassisGimbalManual::checkReferee();       
  shooter_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_);
  self_inspection_event_.update(data_.referee_.referee_data_.game_status_.game_progress_ == 2);
  game_start_event_.update(data_.referee_.referee_data_.game_status_.game_progress_ == 4);
}
```

**检查裁判系统：**

发射能量开启：更新主电源发射输出：1

自检：更新比赛进展==2

比赛开始：更新比赛进展==4

```c++
void ChassisGimbalShooterManual::checkKeyboard() {
  ChassisGimbalManual::checkKeyboard();
  e_event_.update(data_.dbus_data_.key_e);
  g_event_.update(data_.dbus_data_.key_g);
  q_event_.update((!data_.dbus_data_.key_ctrl) & data_.dbus_data_.key_q);
  f_event_.update(data_.dbus_data_.key_f);
  b_event_.update((!data_.dbus_data_.key_ctrl) & data_.dbus_data_.key_b);
  ctrl_c_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_v_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_b_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
  shift_event_.update(data_.dbus_data_.key_shift);
  mouse_left_event_.update(data_.dbus_data_.p_l);
  mouse_right_event_.update(data_.dbus_data_.p_r);
}
```

**检查键盘：**更新数据->e/g/q/f/b/ctrlc/ctrlv/ctrlr/ctrlb/shift/鼠标左右键

```c++
void ChassisGimbalShooterManual::sendCommand(const ros::Time &time) {
  ChassisGimbalManual::sendCommand(time);
  shooter_cmd_sender_->sendCommand(time);
}
```

**发送命令：**

ChassisGimbalManual发送命令：底盘命令，速度命令，云台命令

发射命令发布者：发送命令

```c++
void ChassisGimbalShooterManual::remoteControlTurnOff() {
  ChassisGimbalManual::remoteControlTurnOff();
  shooter_cmd_sender_->setZero();
  trigger_calibration_->stop();
}
```

**远程控制关闭：**controller_manager停止主控制器和校准控制器，模式为被动

设置x轴，y轴线速度为0，z轴角速度为0

设置底盘功率限制为实时功率限制

设置云台yaw轴，pitch轴速度为0

发射命令发送者：设置发射速度为0

扳机校准：停止校准

```c++
void ChassisGimbalShooterManual::remoteControlTurnOn() {
  ChassisGimbalManual::remoteControlTurnOn();
  trigger_calibration_->stopController();
}
```

**远程控制开启：**controller_manager打开主控制器，模式为空闲

扳机校准：关闭控制器->如果校准器通信为空则退出；如果校准器通信!=最终值 && 开关是否打开，则controller_manager关闭控制器

```c++
void ChassisGimbalShooterManual::robotDie() {
  ManualBase::robotDie();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
```

**机器人死亡：**若远程控制器打开，controller_manager停止主控制器和校准控制器

设置发射模式为停止

```c++
void ChassisGimbalShooterManual::chassisOutputOn() {
  ChassisGimbalManual::chassisOutputOn();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}
```

**底盘输出开启：**ROS_INFO("Chassis output ON")；设置底盘功率为充电CHARGE

```c++
void ChassisGimbalShooterManual::shooterOutputOn() {
  ChassisGimbalManual::shooterOutputOn();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  trigger_calibration_->reset();
}
```

**发射输出开启：**ROS_INFO("Shooter output ON")；设置发射模式为停止STOP；

扳机校准器重新设置：如果校准器通信不为空，return；校准器通信开始，开关为false；查询通讯，，应答已经校准

```c++
void ChassisGimbalShooterManual::drawUi(const ros::Time &time) {
  ChassisGimbalManual::drawUi(time);
  if (data_.referee_.referee_data_.robot_id_ != rm_common::RobotId::BLUE_HERO
      && data_.referee_.referee_data_.robot_id_ != rm_common::RobotId::RED_HERO)
    trigger_change_ui_->update("target", switch_detection_srv_->getTarget(),
                               shooter_cmd_sender_->getBurstMode(), switch_detection_srv_->getArmorTarget(),
                               switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::RED);
  else
    trigger_change_ui_->update("target", gimbal_cmd_sender_->getEject() ? 1 : 0,
                               shooter_cmd_sender_->getBurstMode(), switch_detection_srv_->getArmorTarget(),
                               switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::RED);
  trigger_change_ui_->update("exposure", switch_detection_srv_->getExposureLevel(), false);
  fixed_ui_->update();
}
```

**画UI：**

* **更新UI改变时间（电容）：**

如果dbus左叶轮数据等于遥控器中间位 && dbus右叶轮数据等于遥控器上升位，则底盘命令发送者更新

功率限制，设置模式为测试（TEST=0）；

* **UI扳机改变更新：**chassis命名空间，底盘命令发送者获取mode

否则UI扳机改变更新：chassis命名空间，底盘命令发送者获取mode，设置功率限制模式为突发（BURST=1），然后设置功率限制模式为充电（CHARGE=3）

* **装甲板UI：**

更新数据：旋转，底盘实时模式 == GYPO &&  z轴角速度 != 0

更新0/1/2/3号装甲执行器实时时间

* **如果裁判系统中机器人id != 蓝方英雄id && != 红方英雄id：**

  * 则UI扳机改变更新：target命名空间，开关检测通讯获取目标；发射命令发布者获取突发模式；开关检测通讯获取装甲板目标；开关检测通讯获取颜色 == 状态改变请求->red

  * 否则UI扳机改变更新：target命名空间，云台命令发送者获取弹出标志(bool)？1 : 0；发射命令发送者获取突发模式；开关检测通讯获取装甲板目标；开关检测通讯获取颜色 == 状态改变请求->red；

    UI扳机改变更新：exposure（暴露）命名空间，开关检测通讯获取暴露等级，false

* **固定UI更新：**图形载体

  * 更新位置->获取射击速度指数->如果裁判系统中机器人id != 蓝方英雄id && != 红方英雄id则

    速度限制为15，返回0；为18，返回1；为30，返回2

  * 设置操作：图形操作为更新UPDATE==2

  * 显示（优先标志）：如果 参数==上一次参数 && 标题==上一次标题 && 内容==上一次内容，则return。

    如果 标题为空 && 内容为空 && 结束角度(9)= (标题+内容)的整形长度；

    加UI：UI队列删除迭代位置（迭代开始数据+（位置-开始数据的定量））；如果有优先标志，则UI队列往后（参数，内容），否则UI队列插入（UI初始位置，参数，内容）

    上一次内容/标题/参数=此时内容/标题/参数

```c++
void ChassisGimbalShooterManual::updateRc() {
  ChassisGimbalManual::updateRc();
  if (shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP) {
    gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  }
}
```

**更新遥控器：**更新左侧开关的数据

如果发射模式不为STOP，则更新跟踪数据阵列、设置子弹速度

```c++
void ChassisGimbalShooterManual::updatePc() {
  ChassisGimbalManual::updatePc();
  if (chassis_cmd_sender_->power_limit_->getState() != rm_common::PowerLimit::CHARGE) {
    if (!data_.dbus_data_.key_shift && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW && std::sqrt(
        std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) > 0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    else if (data_.referee_.referee_data_.capacity_data.chassis_power_ < 1.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}
```

**更新电脑端：**检查键盘；设置云台速率：x轴速度为左操作杆x方向的操作速度的负数 * 云台的位移改变量，y轴线速度为左操作杆y方向的操作速度 *  云台的位移改变量

如果底盘功率限制模式不为充电CHARGE{如果!shift键 && 底盘模式 ==跟随 && sqrt(x轴线速度+y轴线速度)>0，则底盘功率限制状态更新为正常NORMAL；否则如果底盘电源容量<1，则底盘功率限制更新为突发BURST}

```c++
void ChassisGimbalShooterManual::rightSwitchDownRise() {
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
```

**右侧开关下方升起：**模式为空闲；设置底盘模式为FOLLOW；设置x轴，y轴线速度为0，z轴角速度为0；设置云台模式为RATE；设置云台yaw轴，pitch轴速度为0

更新底盘功率限制状态为充电CHARGE

设置发射模式为停止STOP

```c++
void ChassisGimbalShooterManual::rightSwitchMidRise() {
  ChassisGimbalManual::rightSwitchMidRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
```

**右侧开关中间升起：**模式为遥控；设置底盘模式为FOLLOW；设置云台模式为RATE

更新底盘功率限制模式为充电

设置发射模式为停止STOP

```c++
void ChassisGimbalShooterManual::rightSwitchUpRise() {
  ChassisGimbalManual::rightSwitchUpRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
```

**右侧开关中间升起：**模式为遥控；设置底盘模式为FOLLOW；设置云台模式为RATE

更新底盘功率限制模式为充电

设置发射模式为停止STOP

```c++
void ChassisGimbalShooterManual::leftSwitchDownRise() {
  ChassisGimbalManual::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
```

**左侧开关下方上升：**设置云台模式为速度RATE，设置发射模式为停止STOP

```c++
void ChassisGimbalShooterManual::leftSwitchMidRise() {
  ChassisGimbalManual::leftSwitchMidRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}
```

**左侧开关中间上升：**设置云台模式为跟踪TRACK；设置发射模式为准备READY

```c++
void ChassisGimbalShooterManual::leftSwitchUpRise() {
  ChassisGimbalManual::leftSwitchUpRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
}
```

**左侧开关上方上升：**设置云台模式为跟踪TRACK；设置子弹速度

```c++
void ChassisGimbalShooterManual::leftSwitchUpOn(ros::Duration duration) {
  if (duration > ros::Duration(1.)) {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
  } else if (duration < ros::Duration(0.02)) shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  else shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}
```

**左侧开关上方开启（持续时间）：**

如果持续时间>1s，则设置发射模式为发射PUSH，检查云台检测过程中当前云台角度向目标射击的误差。

否则如果持续时间<0.02s，则设置发射模式为发送PUSH

否则设置发射模式为准备READY

```c++
void ChassisGimbalShooterManual::mouseLeftPress() {
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  if (data_.dbus_data_.p_r)
    shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
}
```

**按压鼠标左键：**设置发射模式为发射PUSH；

如果滚动鼠标滚轮，则检查云台检测过程中当前云台角度向目标射击的误差

```c++
void ChassisGimbalShooterManual::mouseRightPress() {
  gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  gimbal_cmd_sender_->updateCost(data_.track_data_array_);
}
```

**按压鼠标右键：**设置子弹速度；更新跟踪数据阵列

```c++
void ChassisGimbalShooterManual::gPress() {
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
    if (!data_.dbus_data_.key_shift) chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  } else {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    vel_cmd_sender_->setAngularZVel(1.0);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}
```

**按压g键：**

如果底盘的状态不为小陀螺 { 则设置底盘的模式为跟随FOLLOW、设置z轴键速度为0；如果没有触发shift键，则更新底盘功率限制为正常NORMAL}

否则设置底盘模式为小陀螺，设置z轴角速度为1，更新底盘功率限制为突发BURST

```c++
void ChassisGimbalShooterManual::ePress() {
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::TWIST) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  } else {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::TWIST);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}
```

**按压e键：**

如果底盘模式为扭转TWIST，则设置底盘模式为跟随FOLLOW

否则设置底盘模式为扭转TWIST，更新底盘功率限制状态为正常NORMAL

```c++
void ChassisGimbalShooterManual::bPress() {
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}
```

**按压b键：**更新底盘功率限制状态为充电CHARGE

```c++
void ChassisGimbalShooterManual::wPress() {
  ChassisGimbalManual::wPress();
  if ((data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO
      || data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO) && gimbal_cmd_sender_->getEject()) {
    gimbal_cmd_sender_->setEject(false);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}
```

**按压w键：**x方向的位移改变量 = 若位移量>=1，则位移改变量为1，否则为位移改变量+1

如果（裁判系统中机器人id ==蓝方英雄 || 裁判系统中机器人id==红方英雄）&&  是否有弹出标志，则设置弹出标志为false，设置底盘模式为跟随FOLLOW，更新底盘功率限制模式为正常NORMAL

```c++
void ChassisGimbalShooterManual::aPress() {
  ChassisGimbalManual::aPress();
  if ((data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO
      || data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO) && gimbal_cmd_sender_->getEject()) {
    gimbal_cmd_sender_->setEject(false);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}
```

**按压a键：**y方向的位移改变量 = 若位移量>=1，则位移改变量为1，否则为位移改变量+1

如果（裁判系统中机器人id ==蓝方英雄 || 裁判系统中机器人id==红方英雄）&&  是否有弹出标志，则设置弹出标志为false，设置底盘模式为跟随FOLLOW，更新底盘功率限制模式为正常NORMAL

```c++
void ChassisGimbalShooterManual::sPress() {
  ChassisGimbalManual::sPress();
  if ((data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO
      || data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO) && gimbal_cmd_sender_->getEject()) {
    gimbal_cmd_sender_->setEject(false);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}
```

**按压s键：**x方向的位移改变量 = 若位移量<=-1，则位移改变量为-1，否则为位移改变量-1

如果（裁判系统中机器人id ==蓝方英雄 || 裁判系统中机器人id==红方英雄）&&  是否有弹出标志，则设置弹出标志为false，设置底盘模式为跟随FOLLOW，更新底盘功率限制模式为正常NORMAL

```c++
void ChassisGimbalShooterManual::dPress() {
  ChassisGimbalManual::dPress();
  if ((data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO
      || data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO) && gimbal_cmd_sender_->getEject()) {
    gimbal_cmd_sender_->setEject(false);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}
```

**按压d键：**y方向的位移改变量 = 若位移量<=-1，则位移改变量为-1，否则为位移改变量-1

如果（裁判系统中机器人id ==蓝方英雄 || 裁判系统中机器人id==红方英雄）&&  是否有弹出标志，则设置弹出标志为false，设置底盘模式为跟随FOLLOW，更新底盘功率限制模式为正常NORMAL

```c++
void ChassisGimbalShooterManual::shiftPress() {
  if (chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::FOLLOW) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.);
  }
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}
```

**按压shift键：**如果底盘模式不为跟随FOLLOW，则设置底盘模式为跟随FOLLOW，设置z轴角速度为0

更新底盘功率限制为突发BURST

```c++
void ChassisGimbalShooterManual::shiftRelease() {
  if (chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::GYRO)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
}
```

**释放shift键：**如果底盘模式不为小陀螺，则更新底盘功率限制为正常NORMAL

```c++
void ChassisGimbalShooterManual::ctrlCPress() {
  switch_detection_srv_->switchArmorTargetType();
  switch_detection_srv_->callService();
}
```

**按压ctrlC键：**选择检测通信->选择目标装甲类型==ARMOR_ALL(0u)

选择检测通讯->呼叫通讯

```c++
void ChassisGimbalShooterManual::ctrlVPress() {
  switch_detection_srv_->switchEnemyColor();
  switch_detection_srv_->callService();
}
```

**按压ctrlV键：**选择检测通讯->选择敌军机器人颜色==RED

选择检测通讯->呼叫通讯

```c++
void ChassisGimbalShooterManual::ctrlRPress() {
  if (data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO
      || data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    gimbal_cmd_sender_->setEject(true);
  } else {
    switch_detection_srv_->switchTargetType();
    switch_detection_srv_->callService();
    if (switch_detection_srv_->getTarget())
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
}
```

**按压ctrlR键：**

如果裁判系统中机器人id ==蓝方英雄 || 裁判系统中机器人id==红方英雄，则设置底盘模式为小陀螺GYRO，更新底盘功率限制模式为突发BURST，设置弹出标志为true

否则{选择检测通讯->选择目标类型为==ARMOR(0u)，选择检测通讯->呼叫通讯；如果选择通讯->获取到目标，则设置底盘模式为小陀螺GYRO，否则设置底盘模式为跟随FOLLOW}

```c++
void ChassisGimbalShooterManual::ctrlBPress() {
  switch_detection_srv_->switchExposureLevel();
  switch_detection_srv_->callService();
}
```

**按压ctrlB键：**选择检测通讯->选择暴露等级：通讯请求暴露等级 = EXPOSURE_LEVEL_4(4u) ? EXPOSURE_LEVEL_0(0u) : 通讯请求暴露等级+ 1
