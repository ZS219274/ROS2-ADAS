#include "pid_controller.h"
#include <algorithm>
#include <cmath>

namespace Control {
PIDController::PIDController(double kp, double ki, double kd, double dt,
                               double max_output, double min_output)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      dt_(dt),
      max_output_(max_output),
      min_output_(min_output),
      integral_(0.0),
      prev_error_(0.0),
      first_run_(true) {}

double PIDController::compute(double error) {
  // 比例项
  double proportional = kp_ * error;

  // 积分项（使用梯形积分）
  if (!first_run_) {
    integral_ += error * dt_;
    // 防止积分饱和
    double integral_term = ki_ * integral_;
    if (integral_term > max_output_) {
      integral_ = max_output_ / ki_;
    } else if (integral_term < min_output_) {
      integral_ = min_output_ / ki_;
    }
  }

  // 微分项
  double derivative = 0.0;
  if (!first_run_) {
    derivative = kd_ * (error - prev_error_) / dt_;
  } else {
    first_run_ = false;
  }

  // 计算总输出
  double output = proportional + ki_ * integral_ + derivative;

  // 限制输出
  output = std::clamp(output, min_output_, max_output_);

  // 保存当前误差
  prev_error_ = error;

  return output;
}

void PIDController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_run_ = true;
}

void PIDController::setParameters(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double min_output, double max_output) {
  min_output_ = min_output;
  max_output_ = max_output;
}

// PIDControllerWrapper 实现
PIDControllerWrapper::PIDControllerWrapper(const ControlConfigStruct& config) {
  // 初始化PID控制器
  const auto& lateral_config = config.lateral_;
  const auto& longitudinal_config = config.longitudinal_;
  double dt = config.dt_;

  lateral_pid_ = std::make_unique<PIDController>(
      lateral_config.kp_, lateral_config.ki_, lateral_config.kd_, dt,
      lateral_config.max_output_, lateral_config.min_output_);

  longitudinal_pid_ = std::make_unique<PIDController>(
      longitudinal_config.kp_, longitudinal_config.ki_,
      longitudinal_config.kd_, dt, longitudinal_config.max_output_,
      longitudinal_config.min_output_);

  target_speed_ = 10.0;  // 默认目标速度
}

void PIDControllerWrapper::compute_control_inputs(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state,
    int closest_idx, int lookahead_idx,
    double& angular_velocity, double& acceleration) {
  
  // 计算横向误差和航向角误差
  double cross_track_error = compute_cross_track_error(
      trajectory, vehicle_state, closest_idx);
  double heading_error = compute_heading_error(
      trajectory, vehicle_state, lookahead_idx);

  // 横向控制：使用航向角误差和横向误差的组合
  double lateral_error = heading_error + 0.5 * cross_track_error;
  angular_velocity = lateral_pid_->compute(lateral_error);

  // 纵向控制：速度跟踪
  double speed_error = target_speed_ - vehicle_state.speed_;
  acceleration = longitudinal_pid_->compute(speed_error);
}

void PIDControllerWrapper::reset() {
  lateral_pid_->reset();
  longitudinal_pid_->reset();
}

double PIDControllerWrapper::compute_cross_track_error(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state,
    int closest_idx) {
  
  const auto& closest_point =
      trajectory->local_trajectory[closest_idx].path_point;

  // 计算车辆到最近轨迹点的向量
  double dx = closest_point.pose.pose.position.x - vehicle_state.pose_x_;
  double dy = closest_point.pose.pose.position.y - vehicle_state.pose_y_;

  // 计算轨迹点的航向角
  tf2::Quaternion qtn(
      closest_point.pose.pose.orientation.x,
      closest_point.pose.pose.orientation.y,
      closest_point.pose.pose.orientation.z,
      closest_point.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

  // 计算横向误差（垂直于轨迹方向的距离）
  double cos_theta = std::cos(yaw);
  double sin_theta = std::sin(yaw);
  double lateral_error = -dx * sin_theta + dy * cos_theta;

  return lateral_error;
}

double PIDControllerWrapper::compute_heading_error(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state,
    int lookahead_idx) {
  
  const auto& target_point =
      trajectory->local_trajectory[lookahead_idx].path_point;

  // 计算目标航向角
  tf2::Quaternion qtn(target_point.pose.pose.orientation.x,
                      target_point.pose.pose.orientation.y,
                      target_point.pose.pose.orientation.z,
                      target_point.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

  // 计算航向角误差（归一化到[-π, π]）
  double heading_error = yaw - vehicle_state.theta_;
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  return heading_error;
}

}  // namespace Control

