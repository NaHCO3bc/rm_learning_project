## shooter_controller

### 四种状态：stop，ready，push，block

### 通过PID算法控制左右摩擦轮和拨弹轮（触发轮），通过设置摩擦轮角速度来设置子弹速度，同时实现挡块检测

#### 参数：

**block_effort:**阻塞功率（触发电机扭矩）

**block_speed:**阻塞速度（角速度）

**block_duration:**阻塞持续时间

**block_overtime:**阻塞超时

**anti_block_angle:**反转角度（弧度制）

**anti_block_threshold:**反阻塞点（弧度制）

**qd_:**摩擦力角速度

1.触发电机扭矩大于block_effort，触发电机角速度小于block_speed时，视为阻塞block_duration，且shooter_controller的状态切换为block

2.进入block状态的时间超过block_overtime，shooter_controller的状态切换至push

3.摩擦轮的反角超过anti_block_threshold，则摩擦轮反转成功 

### 订阅的主题(rm_msgs/ShootCmd)

**控制器状态，子弹速度，射击频率的命令**

```
uint8 STOP = 0
uint8 READY = 1
uint8 PUSH = 2

uint8 SPEED_10M_PER_SECOND = 0
uint8 SPEED_15M_PER_SECOND = 1
uint8 SPEED_16M_PER_SECOND = 2
uint8 SPEED_18M_PER_SECOND = 3
uint8 SPEED_30M_PER_SECOND = 4

uint8 mode
uint8 speed
float64 hz
time stamp
```

## 代码分析

```c++
void Controller::starting(const ros::Time& /*time*/)
{
  state_ = STOP;
  state_changed_ = true;
}
```

**设置初始state**

```c++
void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_ = *cmd_rt_buffer_.readFromRT();
  config_ = *config_rt_buffer.readFromRT();
    if (state_ != cmd_.mode && state_ != BLOCK)
  {
    state_ = cmd_.mode;
    state_changed_ = true;
  }
  if (state_ != STOP)
    setSpeed(cmd_);
  switch (state_)
  {
    case READY:
      ready(period);
      break;
    case PUSH:
      push(time, period);
      break;
    case STOP:
      stop(time, period);
      break;
    case BLOCK:
      block(time, period);
      break;
  }
  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_trigger_.update(time, period);
}
```

**从缓存区读取命令和参数**

识别命令中的mode并设置左右摩擦轮和扳机的运动周期

```c++
void Controller::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_friction_l_.setCommand(0.);
    ctrl_friction_r_.setCommand(0.);
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
  }
}
```

**mode设置为stop状态，使左右摩擦轮静止，获取扳机关节位置**

```c++
void Controller::ready(const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");

    normalize();
  }
}
```

**mode进入ready状态，进行归一化，标准化**

```c++
void Controller::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  if (ctrl_friction_l_.joint_.getVelocity() >= push_qd_threshold_ * ctrl_friction_l_.command_ &&
      ctrl_friction_l_.joint_.getVelocity() > M_PI &&
      ctrl_friction_r_.joint_.getVelocity() <= push_qd_threshold_ * ctrl_friction_r_.command_ &&
      ctrl_friction_r_.joint_.getVelocity() < -M_PI && (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz)
  {  // Time to shoot!!!
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
    last_shoot_time_ = time;
  }
  else
    ROS_DEBUG("[Shooter] Wait for friction wheel");
```

**mode进入push状态，左摩擦轮速度 >= 发射的角加速度点 * 左摩擦轮的命令 && 左摩擦轮的速度 > PI && 右摩擦轮速度 <= 发射的角加速度点 * 右摩擦轮的命令 && 右摩擦轮速度 < PI && 本次发射的时间 - 上次发射的时间 >=  1/发射的频率，即可发射**

**设置扳机命令，扳机的位置 - 2 PI/发射的旋转圈数，获取上次发射时间**

```c++
 // Check block
  if (ctrl_trigger_.joint_.getEffort() < -config_.block_effort &&
      std::abs(ctrl_trigger_.joint_.getVelocity()) < config_.block_speed)
  {
    if (!maybe_block_)
    {
      block_time_ = time;
      maybe_block_ = true;
    }
```

<u>//检查是否阻塞</u>

//maybe block （有可能阻塞）：**扳机关节的力矩 < -参数设置的阻塞力矩 && 扳机关节速度 < 参数设置的阻塞速度**

```c++
 if ((time - block_time_).toSec() >= config_.block_duration)
    {
      state_ = BLOCK;
      state_changed_ = true;
      ROS_INFO("[Shooter] Exit PUSH");
    }
  }
  else
    maybe_block_ = false;
}
```

**实时时间 - 阻塞时间 >= 参数设置的阻塞持续时间，则确认为阻塞**

```c++
void Controller::block(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter BLOCK");
    last_block_time_ = time;
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition() + config_.anti_block_angle);
  }
  if (std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.joint_.getPosition()) <
          config_.anti_block_threshold ||
      (time - last_block_time_).toSec() > config_.block_overtime)
  {
    normalize();

    state_ = PUSH;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit BLOCK");
  }
}
```

**阻塞后，记录上次阻塞的时间；设置扳机命令：扳机所在的位置+参数设置的阻塞反转角度**

**上述扳机命令的位置-扳机关节所在位置 < 参数设置的阻塞反转点 || 实时时间-上次阻塞的时间 > 参数设置的阻塞超时，则进行标准化，归一化，摆脱阻塞，状态转换成push**

```c++
void Controller::setSpeed(const rm_msgs::ShootCmd& cmd)
{
  double qd_des;
  if (cmd_.speed == cmd_.SPEED_10M_PER_SECOND)
    qd_des = config_.qd_10;
  else if (cmd_.speed == cmd_.SPEED_15M_PER_SECOND)
    qd_des = config_.qd_15;
  else if (cmd_.speed == cmd_.SPEED_16M_PER_SECOND)
    qd_des = config_.qd_16;
  else if (cmd_.speed == cmd_.SPEED_18M_PER_SECOND)
    qd_des = config_.qd_18;
  else if (cmd_.speed == cmd_.SPEED_30M_PER_SECOND)
    qd_des = config_.qd_30;
  else
    qd_des = 0.;
  ctrl_friction_l_.setCommand(qd_des + config_.lf_extra_rotat_speed);
  ctrl_friction_r_.setCommand(-qd_des);
}
```

设置速度

```c++
void Controller::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
}
```

标准化，归一化

```c++
void Controller::reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
{
  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer.readFromNonRT();  // config init use yaml
    config.block_effort = init_config.block_effort;
    config.block_speed = init_config.block_speed;
    config.block_duration = init_config.block_duration;
    config.block_overtime = init_config.block_overtime;
    config.anti_block_angle = init_config.anti_block_angle;
    config.anti_block_threshold = init_config.anti_block_threshold;
    config.qd_10 = init_config.qd_10;
    config.qd_15 = init_config.qd_15;
    config.qd_16 = init_config.qd_16;
    config.qd_18 = init_config.qd_18;
    config.qd_30 = init_config.qd_30;
    config.lf_extra_rotat_speed = init_config.lf_extra_rotat_speed;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .block_effort = config.block_effort,
                        .block_speed = config.block_speed,
                        .block_duration = config.block_duration,
                        .block_overtime = config.block_overtime,
                        .anti_block_angle = config.anti_block_angle,
                        .anti_block_threshold = config.anti_block_threshold,
                        .qd_10 = config.qd_10,
                        .qd_15 = config.qd_15,
                        .qd_16 = config.qd_16,
                        .qd_18 = config.qd_18,
                        .qd_30 = config.qd_30,
                        .lf_extra_rotat_speed = config.lf_extra_rotat_speed };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}
```

重新设置