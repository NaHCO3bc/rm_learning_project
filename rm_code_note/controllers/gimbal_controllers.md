# rm_gimbal_controllers

## 三种状态：Rate(速度)，Track(跟踪)，Direct(检测)

**rm_gimbal_controllers根据指令对yaw（偏航）关节和pitch（俯仰）关节进行PID控制。它可以根据检测数据进行移动平均滤波，并根据弹道模型计算、预测和跟踪目标。**

## rm_msgs

#### 订阅的topic：

**command(GimbalCmd):**设置云台模式，yaw和pitch轴旋转速度、跟踪目标、指向目标和坐标系

**/detection(TargetDetectionArray)：**接受视觉识别数据

**/<camera_names>/camera_info(CameraInfo):**确保detection节点接受到新的图像帧并将预测数据发送到检测节点

#### 发布的topic：

**error(GimbalDesError):**弹道模型计算的以C

**track(TrackDataArray):**用于detection节点决定ROI的预测数据

#### 参数：

**detection_topic(string,默认值:/detection):**检测节点获取预测数据的topic名称

**detection_frame(string,默认值:detection):**检测帧的名称

**camera_topic(string,默认:/galaxy_camera/camera_info):**确定detection节点接受到一帧新图像并将预测数据发送给detection节点

**publish_rate:**发布云台误差频率（Hz）

**classis_angular_data_num:**图像传输延迟的时间(s)，用于补偿图像传输延迟的影响

#### 子弹解算器：

**model_desire(visualization_msgs/Marker):**用于可视化所需的子弹轨迹

**model_real(visualization_msgs/Marker):**用于在当前云台角度可视化弹道模型计算的轨迹

visualization_msgs/Marker:http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html

<u>*子弹解算器用于获取子弹落点*</u>

**resistance_coff_qd_:**子弹速度为10/15/16/18/30 m/s时用于子弹解算器的空气阻力系数

**g：**重力加速度

**delay：**shooter接收到射击指令后射击延迟时间(s)，用于补偿发射延迟的影响

**timeout:**子弹解算器的超时时间(s)。用于判断子弹解算器能否计算出子弹落点

#### 移动平均滤波器：

***用于在目标旋转时过滤目标装甲中心***

**is_debug(bool,默认值：true)：**调试状态。为true时，调试数据将被推送到过滤器topic上

**pos_data_num(默认值:20)：**装甲位置数据的数量

**vel_data_num(默认:30)：**装甲位置数据的数量

**gyro_data_num(默认:100)：**目标转速数据的个数

**center_data_num(默认50)：**目标旋转中心位置数据的数量

**center_offset_z：**z轴上的偏移量(m)。用于补偿z轴上滤波效果的误差

#### GimbalCmd

```
uint8 RATE = 0
uint8 TRACK = 1
uint8 DIRECT = 2

time stamp
uint8 mode

# RATE
float64 rate_yaw
float64 rate_pitch

# TRACK/DIRECT
float64 bullet_speed
geometry_msgs/PointStamped target_pos
geometry_msgs/Vector3Stamped target_vel
```

#### GimbalDesError

```
float64 error

time stamp
```

#### TrackDataArray

```
Header header
TrackData[] tracks
```

#### TargetDectionArray

```
std_msgs/Header header
TargetDetection[] detections
```

#### 设置空气阻力系数，重力加速度，延迟时间，时间变化，超时时间

```python
#!/usr/bin/env python
PACKAGE = "rm_gimbal_controllers"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("resistance_coff_qd_10", double_t, 0, "Air resistance divided by mass of 10 m/s", 0.1, 0, 5.0)
gen.add("resistance_coff_qd_15", double_t, 0, "Air resistance divided by mass of 10 m/s", 0.1, 0, 5.0)
gen.add("resistance_coff_qd_16", double_t, 0, "Air resistance divided by mass of 10 m/s", 0.1, 0, 5.0)
gen.add("resistance_coff_qd_18", double_t, 0, "Air resistance divided by mass of 10 m/s", 0.1, 0, 5.0)
gen.add("resistance_coff_qd_30", double_t, 0, "Air resistance divided by mass of 10 m/s", 0.1, 0, 5.0)
gen.add("g", double_t, 0, "Air resistance divided by mass", 9.8, 9.6, 10.0)
gen.add("delay", double_t, 0, "Delay of bullet firing", 0.0, 0, 0.5)
gen.add("dt", double_t, 0, "Iteration interval", 0.01, 0.0001, 0.1)
gen.add("timeout", double_t, 0, "Flight time exceeded", 2.0, 0.5, 3.0)

exit(gen.generate(PACKAGE, "bullet_solver", "BulletSolver"))
```

## 代码分析

### bullet_solver.cpp

```c++
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  publish_rate_ = getParam(controller_nh, "publish_rate", 50);
//设置发布云台误差频率
  config_ = { .resistance_coff_qd_10 = getParam(controller_nh, "resistance_coff_qd_10", 0.),
              .resistance_coff_qd_15 = getParam(controller_nh, "resistance_coff_qd_15", 0.),
              .resistance_coff_qd_16 = getParam(controller_nh, "resistance_coff_qd_16", 0.),
              .resistance_coff_qd_18 = getParam(controller_nh, "resistance_coff_qd_18", 0.),
              .resistance_coff_qd_30 = getParam(controller_nh, "resistance_coff_qd_30", 0.),
              .g = getParam(controller_nh, "g", 0.),
              .delay = getParam(controller_nh, "delay", 0.),
              .dt = getParam(controller_nh, "dt", 0.),
              .timeout = getParam(controller_nh, "timeout", 0.) };
  config_rt_buffer_.initRT(config_);
//设置一系列参数
  marker_desire_.header.frame_id = "map";
  marker_desire_.ns = "model";
  marker_desire_.action = visualization_msgs::Marker::ADD;
  marker_desire_.type = visualization_msgs::Marker::POINTS;
  marker_desire_.scale.x = 0.02;
  marker_desire_.scale.y = 0.02;
  marker_desire_.color.r = 1.0;
  marker_desire_.color.g = 0.0;
  marker_desire_.color.b = 0.0;
  marker_desire_.color.a = 1.0;
//可视化所需弹道轨迹
  marker_real_ = marker_desire_;
  marker_real_.color.r = 0.0;
  marker_real_.color.g = 1.0;
//在当前云台角度可视化弹道模型计算的轨迹
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  path_desire_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_desire", 10));
  path_real_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_real", 10));
}

```

**设置一系列参数**

```c++
double BulletSolver::getResistanceCoefficient(double bullet_speed) const
{
  // bullet_speed have 5 value:10,15,16,18,30
  double resistance_coff;
  if (bullet_speed < 12.5)
    resistance_coff = config_.resistance_coff_qd_10;
  else if (bullet_speed < 15.5)
    resistance_coff = config_.resistance_coff_qd_15;
  else if (bullet_speed < 17)
    resistance_coff = config_.resistance_coff_qd_16;
  else if (bullet_speed < 24)
    resistance_coff = config_.resistance_coff_qd_18;
  else
    resistance_coff = config_.resistance_coff_qd_30;
  return resistance_coff;
}
```

**设置空气阻力系数**

```c++
bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed)
{
  config_ = *config_rt_buffer_.readFromRT();
  target_pos_ = pos;
  bullet_speed_ = bullet_speed;
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;
```

**设置参数：从缓存区读取实时参数；获取目标位置、子弹速度；三目运算得到空气阻力系数**

```c++
  int count{};
  double temp_z = pos.z;
  double target_rho;
  double error = 999;
  while (error >= 0.001)
  {
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
    target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    double fly_time =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                    config_.g * fly_time / resistance_coff_;

    target_pos_.x = pos.x + vel.x * (config_.delay + fly_time);
    target_pos_.y = pos.y + vel.y * (config_.delay + fly_time);
    target_pos_.z = pos.z + vel.z * (config_.delay + fly_time);

    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    double error_theta = target_yaw - output_yaw_;
    double error_z = target_pos_.z - real_z;
    temp_z += error_z;
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    count++;

    if (count >= 20 || std::isnan(error))
      return false;
  }
  return true;
}
```

**算法：解算子弹**

```c++
void BulletSolver::bulletModelPub(const geometry_msgs::TransformStamped& map2pitch, const ros::Time& time)
{
  marker_desire_.points.clear();
  marker_real_.points.clear();
  double roll{}, pitch{}, yaw{};
  quatToRPY(map2pitch.transform.rotation, roll, pitch, yaw);
  geometry_msgs::Point point_desire{}, point_real{};
  double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
  int point_num = int(target_rho * 20);
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time = (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) /
                      resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_desire.x = rt_bullet_rho * std::cos(output_yaw_) + map2pitch.transform.translation.x;
    point_desire.y = rt_bullet_rho * std::sin(output_yaw_) + map2pitch.transform.translation.y;
    point_desire.z = rt_bullet_z + map2pitch.transform.translation.z;
    marker_desire_.points.push_back(point_desire);
  }
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time =
        (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(-pitch)))) / resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(-pitch) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_real.x = rt_bullet_rho * std::cos(yaw) + map2pitch.transform.translation.x;
    point_real.y = rt_bullet_rho * std::sin(yaw) + map2pitch.transform.translation.y;
    point_real.z = rt_bullet_z + map2pitch.transform.translation.z;
    marker_real_.points.push_back(point_real);
  }
  marker_desire_.header.stamp = time;
  if (path_desire_pub_->trylock())
  {
    path_desire_pub_->msg_ = marker_desire_;
    path_desire_pub_->unlockAndPublish();
  }
  marker_real_.header.stamp = time;
  if (path_real_pub_->trylock())
  {
    path_real_pub_->msg_ = marker_real_;
    path_real_pub_->unlockAndPublish();
  }
}
```

**发布子弹模型**

```c++
double BulletSolver::getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw_real,double pitch_real, double bullet_speed)
{
  config_ = *config_rt_buffer_.readFromRT();
  double resistance_coff = getResistanceCoefficient(bullet_speed);
  double fly_time = (-std::log(1 - std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2)) * resistance_coff /(bullet_speed * std::cos(pitch_real)))) /resistance_coff;
  double last_fly_time{}, target_rho{};
  int count{};
  geometry_msgs::Point target_pos{};
  while (std::abs(fly_time - last_fly_time) > 0.01)
  {
    last_fly_time = fly_time;
    target_pos.x = pos.x + vel.x * (config_.delay + fly_time);
    target_pos.y = pos.y + vel.y * (config_.delay + fly_time);
    target_pos.z = pos.z + vel.z * (config_.delay + fly_time);
    target_rho = std::sqrt(std::pow(target_pos.x, 2) + std::pow(target_pos.y, 2));
    fly_time = (-std::log(1 - target_rho * resistance_coff / (bullet_speed * std::cos(pitch_real)))) / resistance_coff;
    count++;
    if (count >= 20 || std::isnan(fly_time))
      return 999;
  }
  double real_z = (bullet_speed * std::sin(pitch_real) + (config_.g / resistance_coff)) *(1 - std::exp(-resistance_coff * fly_time)) / resistance_coff -config_.g * fly_time / resistance_coff;
  double target_yaw = std::atan2(target_pos.y, target_pos.x);
  double error = std::sqrt(std::pow(target_rho * (std::cos(target_yaw) - std::cos(yaw_real)), 2) +std::pow(target_rho * (std::sin(target_yaw) - std::sin(yaw_real)), 2) +std::pow(target_pos.z - real_z, 2));
  return error;
}
```

**获取云台误差**

```c++
void BulletSolver::reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Bullet Solver] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer_.readFromNonRT();  // config init use yaml
    config.resistance_coff_qd_10 = init_config.resistance_coff_qd_10;
    config.resistance_coff_qd_15 = init_config.resistance_coff_qd_15;
    config.resistance_coff_qd_16 = init_config.resistance_coff_qd_16;
    config.resistance_coff_qd_18 = init_config.resistance_coff_qd_18;
    config.resistance_coff_qd_30 = init_config.resistance_coff_qd_30;
    config.g = init_config.g;
    config.delay = init_config.delay;
    config.dt = init_config.dt;
    config.timeout = init_config.timeout;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .resistance_coff_qd_10 = config.resistance_coff_qd_10,
                        .resistance_coff_qd_15 = config.resistance_coff_qd_15,
                        .resistance_coff_qd_16 = config.resistance_coff_qd_16,
                        .resistance_coff_qd_18 = config.resistance_coff_qd_18,
                        .resistance_coff_qd_30 = config.resistance_coff_qd_30,
                        .g = config.g,
                        .delay = config.delay,
                        .dt = config.dt,
                        .timeout = config.timeout };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
```

**重新配置**

### gimbal_base.cpp

```c++
void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = RATE;
  state_changed_ = true;
}
```

**开始运行，初始模式为rate**

```c++
void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  try
  {
    map2pitch_ = robot_state_handle_.lookupTransform("map", ctrl_pitch_.joint_urdf_->child_link_name, time);
    map2base_ = robot_state_handle_.lookupTransform("map", ctrl_yaw_.joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
```

**从缓存区读取实时数据；获取yaw和pitch轴的位置**

```c++
 if (state_ != cmd_gimbal_.mode)
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case RATE:
      rate(time, period);
      break;
    case TRACK:
      track(time);
      break;
    case DIRECT:
      direct(time);
      break;
  }
  moveJoint(time, period);
}
```

**如果当前模式与命令的mode不同，则更换为命令的mode；返回相应的时间或周期**

```c++
void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion map2base, map2last_des, map2gimbal_des;
  tf2::Quaternion base2gimbal_des;
  double pitch_last, yaw_last;
  tf2::fromMsg(map2base_.transform.rotation, map2base);
  tf2::fromMsg(map2gimbal_des_.transform.rotation, map2last_des);
  map2gimbal_des.setRPY(0, pitch_des, yaw_des);
  base2gimbal_des = map2base.inverse() * map2gimbal_des;
  tf2::Quaternion base2last_des = map2base.inverse() * map2last_des;
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  double base2gimbal_last_des_pitch, base2gimbal_last_des_yaw;
  quatToRPY(map2gimbal_des_.transform.rotation, roll_temp, pitch_last, yaw_last);
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
  quatToRPY(toMsg(base2last_des), roll_temp, base2gimbal_last_des_pitch, base2gimbal_last_des_yaw);
  double pitch_real_des, yaw_real_des;

  if (!setDesIntoLimit(pitch_real_des, pitch_des, pitch_last, base2gimbal_current_des_pitch, base2gimbal_last_des_pitch,
                       ctrl_pitch_.joint_urdf_))
  {
    double yaw_temp;
    tf2::Quaternion base2new_des;
    base2new_des.setRPY(0, ctrl_pitch_.getPosition(), base2gimbal_current_des_yaw);
    quatToRPY(toMsg(map2base * base2new_des), roll_temp, pitch_real_des, yaw_temp);
  }

  if (!setDesIntoLimit(yaw_real_des, yaw_des, yaw_last, base2gimbal_current_des_yaw, base2gimbal_last_des_yaw,
                       ctrl_yaw_.joint_urdf_))
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    base2new_des.setRPY(0, base2gimbal_current_des_pitch, ctrl_yaw_.getPosition());
    quatToRPY(toMsg(map2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
  }

  map2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des);
  map2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controllers");
}
```

**设置目标**

```c++
void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    map2gimbal_des_.transform.rotation = map2pitch_.transform.rotation;
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controllers");
  }
  else
  {
    double roll{}, pitch{}, yaw{};
      //roll滚转角
    quatToRPY(map2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
  }
}
```

**rate模式：云台tf轴转动圈数 = pitch轴转动圈数，记录当前云台角度向目标射击的误差=time**

**非rate模式：将四元数转换成欧拉角；设置目标（时间，yaw轴目标=yaw轴+周期 * 命令中云台的yaw轴，pitch轴目标=pitch轴+周期 * 命令中云台的pitch轴）**

![这里写图片描述](https://img-blog.csdn.net/20170424094120405?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvaGVyb2Fjb29s/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

```c++
void Controller::track(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  double roll_real, pitch_real, yaw_real;
  quatToRPY(map2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  double yaw_compute = yaw_real;
  double pitch_compute = -pitch_real;
  geometry_msgs::Point target_pos = cmd_gimbal_.target_pos.point;
  geometry_msgs::Vector3 target_vel = cmd_gimbal_.target_vel.vector;
```

**mode为track：将四元数转换成欧拉角；yaw轴估算值等于真实值，pitch轴估算值等于真实值；获取目标的位置和速度**

```c++
try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(target_pos, target_pos,
                       robot_state_handle_.lookupTransform("map", cmd_gimbal_.target_pos.header.frame_id,                                                           cmd_gimbal_.target_pos.header.stamp));
    
    if (!cmd_gimbal_.target_vel.header.frame_id.empty())
      tf2::doTransform(target_vel, target_vel,
                       robot_state_handle_.lookupTransform("map", cmd_gimbal_.target_vel.header.frame_id,                                                           cmd_gimbal_.target_vel.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
```

**尝试子弹解算**

```c++
 bool solve_success = bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed);

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (error_pub_->trylock())
    {
      double error =
          bullet_solver_->getGimbalError(target_pos, target_vel, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
      error_pub_->msg_.stamp = time;
      error_pub_->msg_.error = solve_success ? error : 1.0;
      error_pub_->unlockAndPublish();
    }
    bullet_solver_->bulletModelPub(map2pitch_, time);
    last_publish_time_ = time;
  }

  if (solve_success)
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  else
  {
    map2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(map2gimbal_des_, "rm_gimbal_controllers");
  }
}
```

**若解算成功，设置目标的yaw轴和pitch轴位置；若失败，累积当前云台角度向目标射击的误差time**

```c++
void Controller::direct(const ros::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  geometry_msgs::Point aim_point_map = cmd_gimbal_.target_pos.point;
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(aim_point_map, aim_point_map,
                       robot_state_handle_.lookupTransform("map", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = std::atan2(aim_point_map.y - map2pitch_.transform.translation.y,
                          aim_point_map.x - map2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_map.z - map2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_map.x - map2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_map.y - map2pitch_.transform.translation.y, 2)));
  setDes(time, yaw, pitch);
}
```

**mode为direct：目标关节地图为命令设置的目标位置关节；若目标位置帧的id不为空，转换坐标。**

```c++
bool Controller::setDesIntoLimit(double& real_des, double current_des, double last_des, double base2gimbal_current_des,
                                 double base2gimbal_last_des, const urdf::JointConstSharedPtr& joint_urdf)
{
  double upper_limit, lower_limit;
  upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;
  if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
    real_des = current_des;
  else if ((base2gimbal_last_des <= upper_limit && base2gimbal_last_des >= lower_limit) ||
           (angles::two_pi_complement(base2gimbal_last_des) <= upper_limit &&
            angles::two_pi_complement(base2gimbal_last_des) >= lower_limit))
    real_des = last_des;
  else
    return false;
  return true;
}
```

**设置目标限位**

```c++
//移动关节
void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
  gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
  gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
  gyro.z = imu_sensor_handle_.getAngularVelocity()[2];

  geometry_msgs::TransformStamped base_frame2des;
  try
  {
    base_frame2des =
        robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, time);
    tf2::doTransform(gyro, angular_vel_pitch,
                     robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                         imu_sensor_handle_.getFrameId(), time));
    tf2::doTransform(gyro, angular_vel_yaw,
                     robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->child_link_name,
                                                         imu_sensor_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des);

  double pitch_vel_target, yaw_vel_target;
  if (state_ == RATE)
  {
    pitch_vel_target = cmd_gimbal_.rate_pitch;
    yaw_vel_target = cmd_gimbal_.rate_yaw;
  }
  else
  {
    pitch_vel_target = 0;
    yaw_vel_target = 0;
  }
  ctrl_yaw_.setCommand(yaw_des, yaw_vel_target + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
  ctrl_pitch_.setCommand(pitch_des, pitch_vel_target + ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand() + feedForward(time));
}
//前馈控制
double Controller::feedForward(const ros::Time& time)
{
  Eigen::Vector3d gravity(0, 0, -gravity_);
  tf2::doTransform(gravity, gravity,
                   robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name, "map", time));
  Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z);
  double feedforward = -mass_origin.cross(gravity).y();
  if (enable_gravity_compensation_)
  {
    Eigen::Vector3d gravity_compensation(0, 0, gravity_);
    tf2::doTransform(gravity_compensation, gravity_compensation,
                     robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                         ctrl_pitch_.joint_urdf_->parent_link_name, time));
    feedforward -= mass_origin.cross(gravity_compensation).y();
  }
  return feedforward;
}

```

