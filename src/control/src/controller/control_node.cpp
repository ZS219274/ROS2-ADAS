#include "control_node.h"

namespace Control {
ControlNode::ControlNode()
    : Node("control_node")  // 控制节点
{
  RCLCPP_INFO(this->get_logger(), "control_node已创建");

  // 读取配置文件
  control_config_ = std::make_unique<ConfigReader>();
  control_config_->read_vehicles_config();

  // 初始化主车（MainCar 会使用 Planning::ConfigReader 读取 planning 模块的配置）
  main_car_ = std::make_shared<MainCar>();

  // 从 control 配置初始化车辆状态
  vehicle_state_.pose_x_ = control_config_->main_car().pose_x_;
  vehicle_state_.pose_y_ = control_config_->main_car().pose_y_;
  vehicle_state_.theta_ = control_config_->main_car().pose_theta_;
  vehicle_state_.speed_ = control_config_->main_car().speed_ori_;

  // 根据配置选择控制器类型
  const auto& control_config = control_config_->control();
  RCLCPP_INFO(this->get_logger(), "配置的控制算法类型: %d (0-PID, 1-MPC, 2-LQR)", 
              control_config.type_);
  std::string controller_type_name;
  switch (static_cast<ControlType>(control_config.type_)) {
    case ControlType::KPID: {
      controller_ = std::make_unique<PIDControllerWrapper>(control_config);
      controller_type_name = "PID控制器";
      RCLCPP_INFO(this->get_logger(), "使用PID控制器");
      break;
    }
    case ControlType::KMPC: {
      controller_ = std::make_unique<MPCController>(control_config);
      controller_type_name = "MPC控制器";
      RCLCPP_INFO(this->get_logger(), "使用MPC控制器");
      break;
    }
    case ControlType::KLQR: {
      controller_ = std::make_unique<LQRController>(control_config);
      controller_type_name = "LQR控制器";
      RCLCPP_INFO(this->get_logger(), "使用LQR控制器");
      break;
    }
    default: {
      RCLCPP_WARN(this->get_logger(), 
                  "未知的控制类型 %d，默认使用PID控制器", 
                  control_config.type_);
      controller_ = std::make_unique<PIDControllerWrapper>(control_config);
      controller_type_name = "PID控制器（默认）";
      break;
    }
  }

  // 变换广播器
  broadcaster_ = std::make_shared<TransformBroadcaster>(this);

  // 创建轨迹订阅器（planning模块在planning命名空间下发布local_trajectory）
  // 使用绝对话题名，避免命名空间问题
  local_trajectory_sub_ = this->create_subscription<LocalTrajectory>(
      "/planning/local_trajectory", 10,
      std::bind(&ControlNode::trajectory_callback, this, _1));
  
  RCLCPP_INFO(this->get_logger(), "已订阅话题: /planning/local_trajectory");

  // 初始化时间
  first_control_ = true;
  last_control_time_ = this->now();
  last_trajectory_time_ = this->now();
  trajectory_received_count_ = 0;

  // 创建诊断定时器，每2秒检查一次是否收到轨迹消息
  diagnostic_timer_ = this->create_wall_timer(
      2s, std::bind(&ControlNode::diagnostic_timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "控制节点初始化完成 - 控制算法类型: %s, "
              "初始位置: (%.2f, %.2f), 航向角: %.2f, 速度: %.2f",
              controller_type_name.c_str(),
              vehicle_state_.pose_x_, vehicle_state_.pose_y_,
              vehicle_state_.theta_, vehicle_state_.speed_);
  RCLCPP_INFO(this->get_logger(), "等待接收轨迹数据...");
}

void ControlNode::trajectory_callback(
    const LocalTrajectory::SharedPtr trajectory) {
  // 更新收到轨迹的时间
  last_trajectory_time_ = this->now();
  trajectory_received_count_++;
  
  RCLCPP_INFO(this->get_logger(), "[轨迹回调] 收到轨迹数据 #%d，轨迹点数: %ld", 
              trajectory_received_count_, trajectory->local_trajectory.size());
  
  // 检查轨迹是否有效
  const int trajectory_size = trajectory->local_trajectory.size();
  if (trajectory_size < 3) {
    RCLCPP_WARN(this->get_logger(), "轨迹点数小于3,无法处理");
    return;
  }

  // 计算控制周期
  rclcpp::Time current_time = this->now();
  double dt = 0.0;
  if (!first_control_) {
    dt = (current_time - last_control_time_).seconds();
    // 如果时间间隔过大，重置控制器
    if (dt > 1.0) {
      RCLCPP_WARN(this->get_logger(),
                  "控制周期过大(%.2f秒)，重置控制器", dt);
      controller_->reset();
      first_control_ = true;
    }
  } else {
    dt = control_config_->control().dt_;
    first_control_ = false;
  }
  last_control_time_ = current_time;

  // 查找最近的轨迹点
  int closest_idx = find_closest_point(trajectory);
  if (closest_idx < 0) {
    RCLCPP_WARN(this->get_logger(), "未找到最近的轨迹点");
    return;
  }

  // 查找前视点
  int lookahead_idx = find_lookahead_point(trajectory, closest_idx);
  if (lookahead_idx < 0) {
    lookahead_idx = closest_idx;  // 如果找不到前视点，使用最近点
  }

  // 计算控制输入
  double angular_velocity = 0.0;  // 角速度（rad/s）
  double acceleration = 0.0;       // 加速度（m/s²）
  controller_->compute_control_inputs(trajectory, vehicle_state_,
                                      closest_idx, lookahead_idx,
                                      angular_velocity, acceleration);

  // 更新车辆状态（使用车辆运动学模型，使用实际时间间隔）
  update_vehicle_state(angular_velocity, acceleration, dt);

  // 广播坐标变换
  broadcast_transform();

  // 记录日志
  const auto& target_point =
      trajectory->local_trajectory[lookahead_idx].path_point;
  RCLCPP_INFO(this->get_logger(),
              "控制输出: pos:(%.2f, %.2f), theta:%.2f, speed:%.2f, "
              "angular_vel:%.3f, accel:%.3f, closest_idx:%d, lookahead_idx:%d",
              vehicle_state_.pose_x_, vehicle_state_.pose_y_,
              vehicle_state_.theta_, vehicle_state_.speed_, angular_velocity,
              acceleration, closest_idx, lookahead_idx);
}

int ControlNode::find_closest_point(
    const LocalTrajectory::SharedPtr trajectory) {
  const int trajectory_size = trajectory->local_trajectory.size();
  double min_dis = std::numeric_limits<double>::max();
  int closest_index = -1;

  for (int i = 0; i < trajectory_size; i++) {
    double dis = std::hypot(
        trajectory->local_trajectory[i].path_point.pose.pose.position.x -
            vehicle_state_.pose_x_,
        trajectory->local_trajectory[i].path_point.pose.pose.position.y -
            vehicle_state_.pose_y_);
    if (dis < min_dis) {
      min_dis = dis;
      closest_index = i;
    }
  }

  return closest_index;
}

int ControlNode::find_lookahead_point(
    const LocalTrajectory::SharedPtr trajectory, int closest_idx) {
  const int trajectory_size = trajectory->local_trajectory.size();
  const double lookahead_dist = control_config_->control().lookahead_distance_;

  // 从最近点开始向前查找，找到距离接近前视距离的点
  for (int i = closest_idx; i < trajectory_size; i++) {
    double dis = std::hypot(
        trajectory->local_trajectory[i].path_point.pose.pose.position.x -
            vehicle_state_.pose_x_,
        trajectory->local_trajectory[i].path_point.pose.pose.position.y -
            vehicle_state_.pose_y_);
    if (dis >= lookahead_dist) {
      return i;
    }
  }

  // 如果找不到，返回最后一个点
  return trajectory_size - 1;
}


void ControlNode::update_vehicle_state(double angular_velocity,
                                       double acceleration, double dt) {
  // 使用实际的时间间隔更新状态，而不是固定的配置值
  // 限制dt的范围，避免异常值
  dt = std::max(0.001, std::min(dt, 0.5));  // 限制在0.001到0.5秒之间

  // 更新速度
  vehicle_state_.speed_ += acceleration * dt;
  vehicle_state_.speed_ = std::max(0.0, vehicle_state_.speed_);  // 速度不能为负

  // 更新航向角
  vehicle_state_.theta_ += angular_velocity * dt;
  // 归一化航向角到[-π, π]
  while (vehicle_state_.theta_ > M_PI) vehicle_state_.theta_ -= 2.0 * M_PI;
  while (vehicle_state_.theta_ < -M_PI) vehicle_state_.theta_ += 2.0 * M_PI;

  // 更新位置（使用车辆运动学模型）
  vehicle_state_.pose_x_ += vehicle_state_.speed_ * std::cos(vehicle_state_.theta_) * dt;
  vehicle_state_.pose_y_ += vehicle_state_.speed_ * std::sin(vehicle_state_.theta_) * dt;
}

void ControlNode::broadcast_transform() {
  // 创建消息对象
  TransformStamped transform_data;
  transform_data.header.stamp = this->now();
  transform_data.header.frame_id = control_config_->pnc_map().frame_;
  // 使用main_car的child_frame，与car_move_cmd保持一致
  // 注意：rviz2中显示的车辆模型需要匹配这个frame_id
  transform_data.child_frame_id = main_car_->child_frame();

  // 使用控制算法计算的位置和航向角
  transform_data.transform.translation.x = vehicle_state_.pose_x_;
  transform_data.transform.translation.y = vehicle_state_.pose_y_;
  transform_data.transform.translation.z = 0.0;

  // 将航向角转换为四元数
  tf2::Quaternion qtn;
  qtn.setRPY(0.0, 0.0, vehicle_state_.theta_);
  transform_data.transform.rotation.x = qtn.getX();
  transform_data.transform.rotation.y = qtn.getY();
  transform_data.transform.rotation.z = qtn.getZ();
  transform_data.transform.rotation.w = qtn.getW();

  // 广播消息
  broadcaster_->sendTransform(transform_data);
  
  // 调试信息（降低频率，避免日志过多）
  static int count = 0;
  if (count++ % 10 == 0) {
    RCLCPP_DEBUG(this->get_logger(),
                "广播TF: frame_id=%s, child_frame_id=%s, pos=(%.2f, %.2f), theta=%.2f",
                transform_data.header.frame_id.c_str(),
                transform_data.child_frame_id.c_str(),
                vehicle_state_.pose_x_, vehicle_state_.pose_y_,
                vehicle_state_.theta_);
  }
}

void ControlNode::diagnostic_timer_callback() {
  rclcpp::Time current_time = this->now();
  double time_since_last_trajectory = 
      (current_time - last_trajectory_time_).seconds();
  
  // 检查订阅器状态
  size_t publisher_count = local_trajectory_sub_->get_publisher_count();
  
  if (trajectory_received_count_ == 0) {
    // 从未收到过消息
    RCLCPP_WARN(this->get_logger(),
                "[诊断] 启动后未收到任何轨迹消息！已等待 %.1f 秒。"
                "发布者数量: %zu。请检查："
                "1) 规划模块是否正在运行？"
                "2) 话题名称是否正确？(订阅: /planning/local_trajectory)",
                time_since_last_trajectory, publisher_count);
  } else if (time_since_last_trajectory > 1.0) {
    // 超过1秒未收到消息
    RCLCPP_WARN(this->get_logger(),
                "[诊断] 已 %.1f 秒未收到轨迹消息！"
                "已收到消息总数: %d，发布者数量: %zu。"
                "规划模块可能已停止发布轨迹。",
                time_since_last_trajectory, trajectory_received_count_, publisher_count);
  } else {
    // 正常接收消息
    RCLCPP_DEBUG(this->get_logger(),
                 "[诊断] 轨迹接收正常。已收到 %d 条消息，"
                 "距离上次消息 %.3f 秒，发布者数量: %zu",
                 trajectory_received_count_, time_since_last_trajectory, publisher_count);
  }
}
}  // namespace Control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Control::ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
