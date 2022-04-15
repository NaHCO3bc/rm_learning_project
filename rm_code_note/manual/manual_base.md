# manual_base

## data.h（数据）

```c++
class Data {
 public:
  explicit Data(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    // sub
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Data::jointStateCallback, this);
    actuator_state_sub_ =
        nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &Data::actuatorStateCallback, this);
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    track_sub_ =
        nh.subscribe<rm_msgs::TrackDataArray>("/controllers/gimbal_controller/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/controllers/gimbal_controller/error_des", 10,
                                              &Data::gimbalDesErrorCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Data::odomCallback, this);
```

**构造Data函数：**（节点句柄节省内存空间）transfrom的接听者和缓存区

关节状态订阅者，执行器状态订阅者，dbus通讯订阅者，目标跟踪订阅者，云台目标误差订阅者，里程计坐标系订阅者

```c++
    // pub
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    initSerial();
  }
```

创造节点句柄，裁判系统发布者，超级电容发布者，初始化

```c++
void initSerial() {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort("/dev/usbReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen()) return;
    try {
      serial_.open();
    }
    catch (serial::IOException &e) {
      ROS_ERROR("Cannot open referee port");
    }
  }
```

设置超时（限定接受方发回反馈信息的时间间隔，超时未能接受则认定为传出的数据帧已出错或丢失，从而重新发送）设置通信端口（串口），设置通信波特率，判断裁判系统是否已连接。

## input_event.h

```c++
class InputEvent {
 public:
  InputEvent() : last_state_(false) {}
  void setRising(boost::function<void()> handler) { rising_handler_ = std::move(handler); }
  void setFalling(boost::function<void()> handler) { falling_handler_ = std::move(handler); }
  void setActiveHigh(boost::function<void(ros::Duration)> handler) { active_high_handler_ = std::move(handler); }
  void setActiveLow(boost::function<void(ros::Duration)> handler) { active_low_handler_ = std::move(handler); }
```

**定义输入事件函数：**上次的状态为false

设置上升函数，下降函数，设置高电平有效函数，设置低电平有效函数

```c++
  void setEdge(boost::function<void()> rising_handler, boost::function<void()> falling_handler) {
    rising_handler_ = std::move(rising_handler);
    falling_handler_ = std::move(falling_handler);
  }
  void setActive(boost::function<void(ros::Duration)> high_handler, boost::function<void(ros::Duration)> low_handler) {
    active_high_handler_ = std::move(high_handler);
    active_low_handler_ = std::move(low_handler);
  }
```

设置限位（上升/下降处理器）；设置电平有效（高/低电平有效处理器）

```c++
void update(bool state) {
    if (state != last_state_) {
      if (state && rising_handler_)
        rising_handler_();
      else if (!state && falling_handler_)
        falling_handler_();
      last_state_ = state;
      last_change_ = ros::Time::now();
    }
    if (state && active_high_handler_)
      active_high_handler_(ros::Time::now() - last_change_);
    if (!state && active_low_handler_)
      active_low_handler_(ros::Time::now() - last_change_);
  }
```

**更新函数：**根据状态选择上升/下降处理器，选择高/低电平处理器

## manual_base.h

```c++
class ManualBase {
 public:
  explicit ManualBase(ros::NodeHandle &nh);
  enum { PASSIVE, IDLE, RC, PC };
  virtual void run();
```

**构造ManualBase函数：**设置模式：PASSIVE（被动），IDLE（空闲），RC（遥控控制），PC（电脑控制）；定义多态函数run()

```c++
protected:
  void checkSwitch(const ros::Time &time);
  virtual void checkReferee();//检查裁判系统
  virtual void checkKeyboard() {};//检查键盘
  virtual void updateRc();//更新RC端
  virtual void updatePc();//更新PC端
  virtual void sendCommand(const ros::Time &time) = 0;//定义发送命令时间函数
  virtual void drawUi(const ros::Time &time) { data_.referee_.sendUi(time); }//画UI
```

```c++
// Referee
  virtual void chassisOutputOn() { ROS_INFO("Chassis output ON"); }//底盘输出开启
  virtual void gimbalOutputOn() { ROS_INFO("Gimbal output ON"); }//云台输出开启
  virtual void shooterOutputOn() { ROS_INFO("Shooter output ON"); }//发射输出开启
  virtual void robotDie();//机器人死亡
  virtual void robotRevive();//机器人复活
```

```c++
 // Remote Controller
  virtual void remoteControlTurnOff();//远程控制关闭
  virtual void remoteControlTurnOn();//远程控制开启
  virtual void leftSwitchDownRise() {};//左侧开关下方升起
  virtual void leftSwitchMidRise() {};//左侧开关中间升起
  virtual void leftSwitchMidFall() {};//左侧开关中间落下
  virtual void leftSwitchUpRise() {};//左侧开关上方升起
  virtual void rightSwitchDownRise() { state_ = IDLE; }//右侧开关下方升起：模式为空闲
  virtual void rightSwitchMidRise() { state_ = RC; }//右侧开关中间升起：模式为遥控
  virtual void rightSwitchUpRise() { state_ = PC; }//右侧开关上方升起：模式为电脑控制
```

```c++
Data data_;
  ros::NodeHandle nh_;
  rm_common::ControllerManager controller_manager_;

  bool remote_is_open_{};//定义远程连接开启字符
  int state_ = PASSIVE;//定义被动状态
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;
};
```

## manual_base.cpp

```c++
ManualBase::ManualBase(ros::NodeHandle &nh) : data_(nh), nh_(nh), controller_manager_(nh) {
  controller_manager_.startStateControllers();
  right_switch_down_event_.setRising(boost::bind(&ManualBase::rightSwitchDownRise, this));
  right_switch_mid_event_.setRising(boost::bind(&ManualBase::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&ManualBase::rightSwitchUpRise, this));
  left_switch_down_event_.setRising(boost::bind(&ManualBase::leftSwitchDownRise, this));
  left_switch_up_event_.setRising(boost::bind(&ManualBase::leftSwitchUpRise, this));
  left_switch_mid_event_.setEdge(boost::bind(&ManualBase::leftSwitchMidRise, this),
                                 boost::bind(&ManualBase::leftSwitchMidFall, this));
  robot_hp_event_.setEdge(boost::bind(&ManualBase::robotRevive, this), boost::bind(&ManualBase::robotDie, this));
}
```

**定义ManualBase：**左右开关移动性质，机器人受伤命令

```c++
void ManualBase::run() {
  ros::Time time = ros::Time::now();
  try {
    if (data_.serial_.available()) {
      data_.referee_.rx_len_ = (int) data_.serial_.available();
      data_.serial_.read(data_.referee_.rx_buffer_, data_.referee_.rx_len_);
    }
  } catch (serial::IOException &e) {}
  data_.referee_.read();
  checkReferee();
  checkSwitch(time);
  sendCommand(time);
  drawUi(time);
  controller_manager_.update();
  try {
    data_.serial_.write(data_.referee_.tx_buffer_, data_.referee_.tx_len_);
  }
  catch (serial::PortNotOpenedException &e) {}
  data_.referee_.clearBuffer();
}
```

**设置运行函数：**

尝试测试通信串口是否有数据，设置数据长度；读取裁判系统缓存区数据和数据长度；否则检查异常（IOException）

读取数据，检查开关，发送命令，画UI，controller_manager更新数据

尝试写出裁判系统缓存区数据和数据长度；否则检查通信端口（串口）是否打开

清理裁判系统缓存区数据

```c++
void ManualBase::checkReferee() {
  robot_hp_event_.update(data_.referee_.referee_data_.game_robot_status_.remain_hp_ != 0);
}
```

**检查裁判系统：**检查机器人血量不为0

```c++
void ManualBase::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() > 0.3) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() < 0.3) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
  right_switch_down_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN);
  right_switch_mid_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::MID);
  right_switch_up_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::UP);
  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}
```

**检查开关：**

若远程连接打开 && 实时时间-遥控器接受数据时间 > 0.3 ->远程控制器未打开

若远程连接打开 && 实时时间-遥控器接受数据时间 < 0.3 ->远程控制器已打开

更新右侧开关的数据，根据控制器模式，更新数据

```c++
void ManualBase::updateRc() {
  left_switch_down_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
  left_switch_mid_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::MID);
  left_switch_up_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
}

void ManualBase::updatePc() {
  checkKeyboard();
}
```

**状态为RC：**更新左侧开关的数据

**状态为PC：**检查键盘

```c++
void ManualBase::remoteControlTurnOff() {
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
  state_ = PASSIVE;
}
void ManualBase::remoteControlTurnOn() {
  controller_manager_.startMainControllers();
  state_ = IDLE;
}
```

**远程控制器关闭：**controller_manager停止主控制器和校准控制器，模式为被动

**远程控制器打开：**controller_manager打开主控制器，模式为空闲

```c++
void ManualBase::robotRevive() {
  if (remote_is_open_) controller_manager_.startMainControllers();
}

void ManualBase::robotDie() {
  if (remote_is_open_) {
    controller_manager_.stopMainControllers();
    controller_manager_.stopCalibrationControllers();
  }
}
```

**机器人复活：**若远程控制打开，controller_manager打开主控制器

**机器人死亡：**若远程控制器打开，controller_manager停止主控制器和校准控制器