## engineer_manual

## engineer_manual.h

```c++
class EngineerManual : public ChassisGimbalManual {
 public:
  explicit EngineerManual(ros::NodeHandle &nh);
  void run() override;
```

**继承ChassisGimbalManual：**

public：构造函数：EngineerManual；定义运行函数run()

```c++
private:
  void checkKeyboard() override;//检查键盘
  void updateRc() override;//更新遥控器
  void updatePc() override;//更新电脑端
  void sendCommand(const ros::Time &time) override;//发送命令
  void drawUi(const ros::Time &time) override;//画UI
  void remoteControlTurnOff() override;//关闭远程控制
  void chassisOutputOn() override;//底盘输出开启
  void rightSwitchDownRise() override;//右侧开关下方升起
  void rightSwitchMidRise() override;//右侧开关中间升起
  void rightSwitchUpRise() override;//右侧开关上方升起
  void leftSwitchUpFall() {
    runStepQueue("ARM_TEST");
    trigger_change_ui_->update("step", "ARM_TEST");
  }//左侧开关上方升起：运行步骤队列（ARM_TEST）；扳机改变UI：更新（步骤：ARM_TEST）
  void leftSwitchDownFall();//左侧开关下方落下
  void runStepQueue(const std::string &step_queue_name);//运行步骤队列：名称（字符串）
  void actionActiveCallback() { operating_mode_ = MIDDLEWARE; }//主动行为回调函数：操作模式=中间件
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr &feedback);//行为反馈回调函数：常量指针（feedback）
  void actionDoneCallback(const actionlib::SimpleClientGoalState &state,//行为完成回调函数：（常量）简单的客户端目标模式（state）
                          const rm_msgs::EngineerResultConstPtr &result);//工程结果常量指针（result）
```

```c++
 void ctrlCPress() { action_client_.cancelAllGoals(); }//按压ctrlC键：客户端行为->取消所有目标
  void ctrlRPress() {
    runStepQueue("RECOVER");
    trigger_change_ui_->update("step", "RECOVER");
  }//按压ctrlR键：运行步骤队列->恢复(RECOVER)；扳机改变UI->更新：步骤，恢复
  void ctrlZPress() {
    runStepQueue("ARM_FOLD_LOWER");
    trigger_change_ui_->update("step", "ARM_FOLD_LOWER");
  }//按压ctrlZ键：运行步骤队列->机械臂折叠降低；扳机改变UI->更新：步骤，机械臂折叠降低
  void ctrlBPress() {
    target_ = "BIG_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlB键：目标->大BIG；扳机改变UI->更新：步骤，前缀+目标
  void ctrlFPress() {
    target_ = "FLOOR_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlF键：目标->地板FLOOR；扳机改变UI->更新：步骤，前缀+目标
  void ctrlXPress() {
    target_ = "EXCHANGE_";
    prefix_ = "";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlX键：目标->交换EXCHANGE；前缀为"";扳机改变UI->更新：步骤，前缀+目标
  void ctrlGPress() {
    prefix_ = "GRASP_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlG键：前缀->抓住GRASP；扳机改变UI->更新：步骤，前缀+目标
  void ctrlSPress() {
    prefix_ = "STORAGE_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlS键：前缀->储存STORAGE；扳机改变UI->更新：步骤，前缀+目标
  void ctrlDPress() {
    prefix_ = "DRAG_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlD键：前缀->拖/拽DRAG；扳机改变UI->更新：步骤，前缀+目标
  void ctrlQPress() {
    runStepQueue(prefix_ + target_ + "PRE");
    trigger_change_ui_->update("step", prefix_ + target_ + "PRE");
  }//按压ctrlQ键：运行步骤队列->前缀+目标+在...之前"PRE"；扳机改变UI->更新：步骤，前缀+目标+在...之前"PRE"
  void ctrlWPress() {
    runStepQueue(prefix_ + target_ + "PROC");
    trigger_change_ui_->update("step", prefix_ + target_ + "PROC");
  }//按压ctrlW键：运行步骤队列->前缀+目标+程序"PROC"；扳机改变UI->更新：步骤，前缀+目标+程序"PROC"
  void ctrlEPress() {
    runStepQueue(prefix_ + target_ + "AFTER");
    trigger_change_ui_->update("step", prefix_ + target_ + "AFTER");
  }//按压ctrlE键：运行步骤队列->前缀+目标+在...之后"AFTER"；扳机改变UI->更新：步骤，前缀+目标+在...之后"AFTER"
  void ctrlVPress() {
    target_ = "LIGHT_BAR_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }//按压ctrlV键：目标->光栏LIGHT_BAR；扳机改变UI->更新：步骤，前缀+目标
  void shiftWPress() {
    runStepQueue("GIMBAL_FORWARD_UPPER");
    trigger_change_ui_->update("step", "GIMBAL_FORWARD_UPPER");
  }//按压shiftW键：运行步骤队列->云台前方升起；扳机改变UI->更新：步骤，云台前方升起
  void shiftSPress() {
    runStepQueue("GIMBAL_FORWARD_LOWER");
    trigger_change_ui_->update("step", "GIMBAL_FORWARD_LOWER");
  }//按压shiftS键：运行步骤队列->云台前方下降；扳机改变UI->更新：步骤，云台前方下降
  void shiftCPress() { power_on_calibration_->reset(); }//按压shiftC键：通电的校准器->重新设置
  void shiftXPress() {
    sentry_mode_ = sentry_mode_ == 0 ? 1 : 0;
  }//按压shiftX键：哨兵模式 = 哨兵模式 == 0 ? 1 ：0
  void cPress();//按压c键
```

```c++
 enum { MANUAL, MIDDLEWARE };//枚举：MANUAL，MIDDLEWARE（中间件）
  std::string target_, prefix_;//定义字符串target_，prefix_（前缀）
  int operating_mode_, sentry_mode_;//定义操作模式，哨兵模式
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;//定义简单动作客户端
  rm_common::CalibrationQueue *power_on_calibration_{}, *mast_calibration_{}, *arm_calibration_{};//定义校准器队列：通电的校准器，杆校准器，机械臂校准器
  rm_common::JointPositionBinaryCommandSender *mast_command_sender_, *card_command_sender_;//定义关节位置二进制命令发送者：杆命令发送者，卡片命令发送者
//定义按键事件
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_c_event_, ctrl_r_event_, ctrl_f_event_,
      ctrl_z_event_, ctrl_b_event_, ctrl_x_event_, ctrl_v_event_, ctrl_g_event_, ctrl_s_event_, ctrl_d_event_,
      ctrl_q_event_, ctrl_w_event_, ctrl_e_event_, shift_w_event_, shift_s_event_, shift_c_event_, shift_x_event_,
      c_event_;
};
```

## engineer_manual.cpp

```c++
namespace rm_manual {
EngineerManual::EngineerManual(ros::NodeHandle &nh)//定义对象EngineerManual
    : ChassisGimbalManual(nh), operating_mode_(MANUAL), action_client_("/engineer_middleware/move_steps", true) {
  ROS_INFO("Waiting for middleware to start.");//行为客户端："/工程中间件/移动步骤"
  action_client_.waitForServer();//行动客户端->等待通信
  ROS_INFO("Middleware started.");//中间件开始
  // Command sender
  ros::NodeHandle nh_card(nh, "card");//创造节点句柄nh_card，命名空间card
  card_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_card);//定义函数：卡片命令发送者->关节位置二进制命令发送者
  ros::NodeHandle nh_mast(nh, "mast");//创造节点句柄nh_mast，命名空间mast
  mast_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_mast);//定义函数：杆命令发送者->关节位置二进制命令发送者
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;//远程控制有效
  nh.getParam("power_on_calibration", rpc_value);//获取参数：通电的校准器
  power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);//定义函数：通电的校准器->校准器队列
  nh.getParam("mast_calibration", rpc_value);//获取参数：杆校准器
  mast_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);//定义函数：杆校准器->校准器队列
  nh.getParam("arm_calibration", rpc_value);//获取函数：机械臂校准器
  arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);//定义函数：机械臂校准器->校准器队列      
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));//左开关上方事件->设置下降：左开关上方下降
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));//左开关下方事件->设置下降：左开关下方下降
  //ctrlC/R/Z/B/F/X/V/G/S/D/Q/W/E键事件->设置升起：按压ctrlC/R/Z/B/F/X/V/G/S/D/Q/W/E键   
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  ctrl_z_event_.setRising(boost::bind(&EngineerManual::ctrlZPress, this));
  ctrl_b_event_.setRising(boost::bind(&EngineerManual::ctrlBPress, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_x_event_.setRising(boost::bind(&EngineerManual::ctrlXPress, this));
  ctrl_v_event_.setRising(boost::bind(&EngineerManual::ctrlVPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));
  ctrl_s_event_.setRising(boost::bind(&EngineerManual::ctrlSPress, this));
  ctrl_d_event_.setRising(boost::bind(&EngineerManual::ctrlDPress, this));
  ctrl_q_event_.setRising(boost::bind(&EngineerManual::ctrlQPress, this));
  ctrl_w_event_.setRising(boost::bind(&EngineerManual::ctrlWPress, this));
  ctrl_e_event_.setRising(boost::bind(&EngineerManual::ctrlEPress, this));
  //shiftW/S/C/X键事件->设置升起：按压shiftW/S/C/X键
  shift_w_event_.setRising(boost::bind(&EngineerManual::shiftWPress, this));
  shift_s_event_.setRising(boost::bind(&EngineerManual::shiftSPress, this));
  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  shift_x_event_.setRising(boost::bind(&EngineerManual::shiftXPress, this));
  //c键事件->设置升起：按压c键
  c_event_.setRising(boost::bind(&EngineerManual::cPress, this));
  sentry_mode_ = 1;//哨兵模式为1
}
```

```c++
void EngineerManual::run() {
  ChassisGimbalManual::run();
  power_on_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  mast_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  arm_calibration_->update(ros::Time::now());
}
```

**运行函数：**

通电的校准器->更新实时时间，状态!=被动PASSIVE

杆校准器->更新实时时间，状态!=被动PASSIVE

机械臂校准器->更新实时时间

```c++
void EngineerManual::checkKeyboard() {
  ChassisGimbalManual::checkKeyboard();
  ctrl_c_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_z_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
  ctrl_b_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
  ctrl_f_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_f);
  ctrl_x_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_x);
  ctrl_v_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
  ctrl_g_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_g);
  ctrl_s_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_s);
  ctrl_d_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_d);
  ctrl_q_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_q);
  ctrl_w_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_w);
  ctrl_e_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_e);

  shift_w_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_w);
  shift_s_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_s);
  shift_c_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_c);
  shift_x_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_x);

  c_event_.update(data_.dbus_data_.key_c & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
}
```

**检查键盘：**更新键盘事件数据：换位运算

```c++
void EngineerManual::updateRc() {
  ChassisGimbalManual::updateRc();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  left_switch_up_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
}
```

**更新遥控器：**

设置底盘模式：初始RAW

更新左开关上方事件：左叶轮==上方

更新左开关下方事件：左叶轮==下方

```c++
void EngineerManual::updatePc() {
  ChassisGimbalManual::updatePc();
  vel_cmd_sender_->setAngularZVel(-data_.dbus_data_.m_x);
}
```

**更新电脑端：**

速度命令发送者->设置z轴角速度= -鼠标x轴速度

```c++
void EngineerManual::sendCommand(const ros::Time &time) {
  mast_command_sender_->sendCommand(time);
  if (operating_mode_ == MANUAL) {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    card_command_sender_->sendCommand(time);
  }
}
```

**发送命令：**杆命令发送者->发送命令

如果操作模式 == 手册MANUAL，则底盘命令发送者发送命令，速度命令发送者发送命令，卡片命令发送者发送命令

```c++
void EngineerManual::drawUi(const ros::Time &time) {
  ChassisGimbalManual::drawUi(time);
  time_change_ui_->update("effort", time);
  time_change_ui_->update("temperature", time);
  trigger_change_ui_->update("card", 0, card_command_sender_->getState());
  if (data_.referee_.referee_data_.interactive_data.header_data_.data_cmd_id_ == 0x0201
      && data_.referee_.referee_data_.interactive_data.data_ != sentry_mode_)
    data_.referee_.sendInteractiveData(0x0200, data_.referee_.referee_data_.robot_color_ == "blue"
                                               ? rm_common::RobotId::BLUE_SENTRY : rm_common::RED_SENTRY, sentry_mode_);
  trigger_change_ui_->update("sentry", data_.referee_.referee_data_.interactive_data.data_, false);
  flash_ui_->update("calibration", time, power_on_calibration_->isCalibrated());
  if (!data_.joint_state_.name.empty())
    flash_ui_->update("card_warning", time, data_.joint_state_.effort[0] < 1.5);
//    trigger_change_ui_->update("jog", jog_joint_name);
}
```

**画UI：**

* 时间改变UI：更新力矩effort，执行时间

* 时间改变UI：更新温度tempeature，执行时间

* 扳机改变UI：更新卡片，0，卡片命令发送者状态

* 如果命令交互的id==0x0201 && 交互数据 != 哨兵状态：
  * 则发送交互数据->(0x0200，机器人颜色 == blue ? 蓝色哨兵id : 红色哨兵id，哨兵模式)；
  * 扳机改变UI->更新(哨兵命名空间，交互数据，false)；
  * 装甲板闪光UI->更新(校准器命名空间，执行时间，通电校准器->已经校准)；
* 如果关节状态名称为空，则装甲板闪光UI->更新(卡片警告命名空间，执行时间，关节模式力矩数组[0] < 1.5)
* 扳机改变UI：更新(手动，手动关节名称)

```c++
void EngineerManual::remoteControlTurnOff() {
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}
```

**远程控制关闭：**行为客户端->取消所有目标

```c++
void EngineerManual::chassisOutputOn() {
  power_on_calibration_->reset();
  mast_calibration_->reset();
  if (MIDDLEWARE)
    action_client_.cancelAllGoals();
}
```

**底盘输出开启：**

通电校准器重新设置；杆校准器重新设置；

如果有中间件，则行为客户端->取消所有目标

```c++
void EngineerManual::rightSwitchDownRise() {
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
}
```

**右侧开关下方升起：**

设置底盘模式：初始RAW

行为客户端：取消所有目标

```c++
void EngineerManual::rightSwitchMidRise() {
  ChassisGimbalManual::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
```

**右侧开关中间升起：**设置底盘模式：初始RAW

```c++
void EngineerManual::rightSwitchUpRise() {
  ChassisGimbalManual::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
```

**右侧开关上方升起：**设置底盘模式：初始RAW

```c++
void EngineerManual::leftSwitchDownFall() {
  mast_calibration_->reset();
  arm_calibration_->reset();
}
```

**左侧开关下方落下：**

杆校准器重新设置；机械臂校准器重新设置

```c++
void EngineerManual::runStepQueue(const std::string &step_queue_name) {
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = step_queue_name;
  if (action_client_.isServerConnected()) {
    if (operating_mode_ == MANUAL)
      action_client_.sendGoal(goal,
                              boost::bind(&EngineerManual::actionDoneCallback, this, _1, _2),
                              boost::bind(&EngineerManual::actionActiveCallback, this),
                              boost::bind(&EngineerManual::actionFeedbackCallback, this, _1));
    operating_mode_ = MIDDLEWARE;
  } else
    ROS_ERROR("Can not connect to middleware");
}
```

**运行步骤队列：**定义工程机器人目标；获取目标步骤队列名称

如果行为客户端->通信已连接，则

如果操作模式为MANUAL，则行为客户端->设置目标（行为完成回调函数，主动行为回调函数，行为反馈回调函数），操作模式为中间件MIDDLEWARE；否则报错：没有连接到中间件

```c++
void EngineerManual::actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr &feedback) {
  trigger_change_ui_->update("queue", feedback->current_step);
  if (feedback->total_steps != 0)
    time_change_ui_->update("progress", ros::Time::now(), ((double) feedback->finished_step) / feedback->total_steps);
  else
    time_change_ui_->update("progress", ros::Time::now(), 0.);
}
```

**行为反馈回调函数：**定义常量指针

扳机改变UI->更新(队列queue命名空间，反馈->当前步骤)

如果反馈->全部步骤 != 0，则时间改变UI->更新(进展命名空间，执行时间，反馈->完成步骤/所有步骤)，否则时间改变UI更新(进展命名空间，执行时间，0)

```c++
void EngineerManual::actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                                        const rm_msgs::EngineerResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  operating_mode_ = MANUAL;
}
```

**行为完成回调函数：**定义常量指针：简单的客户端目标模式，工程机器人结果

打印：Finished in state(状态字符串)；Result: (完成结果)

操作模式=MANUAL

```c++
void EngineerManual::cPress() {
  if (card_command_sender_->getState()) card_command_sender_->off();
  else card_command_sender_->on();
}
```

**按压c键：**如果卡片命令发送者状态为true，则获取关闭位置、设置状态为false；否则，获取开启位置、设置状态为true
