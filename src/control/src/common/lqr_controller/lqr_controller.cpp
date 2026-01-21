#include "lqr_controller.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace Control {

LQRController::LQRController(const ControlConfigStruct& config)
    : config_(config), gain_computed_(false) {
  // 初始化LQR权重参数
  q_x_ = 1.0;              // x位置权重
  q_y_ = 1.0;              // y位置权重
  q_theta_ = 2.0;          // 航向角权重
  q_v_ = 1.0;              // 速度权重
  r_angular_ = 0.1;        // 角速度权重
  r_acceleration_ = 0.1;   // 加速度权重

  // 设置控制输入限制
  max_angular_velocity_ = config_.lateral_.max_output_;
  min_angular_velocity_ = config_.lateral_.min_output_;
  max_acceleration_ = config_.longitudinal_.max_output_;
  min_acceleration_ = config_.longitudinal_.min_output_;

  // 初始化增益矩阵为2x4零矩阵
  K_.resize(2);
  for (int i = 0; i < 2; ++i) {
    K_[i].resize(4, 0.0);
  }

  // 计算LQR增益矩阵
  compute_lqr_gain();
}

void LQRController::compute_control_inputs(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state,
    int closest_idx, int lookahead_idx,
    double& angular_velocity, double& acceleration) {
  
  // 提取参考状态（使用最近点计算误差，使用前视点作为目标）
  std::vector<double> reference_state_closest = extract_reference_state(
      trajectory, closest_idx);
  std::vector<double> reference_state_lookahead = extract_reference_state(
      trajectory, lookahead_idx);
  
  // 计算轨迹坐标系下的误差（类似PID控制器的方法）
  // 横向误差：垂直于轨迹方向的距离
  double lateral_error = compute_lateral_error(
      trajectory, vehicle_state, closest_idx);
  
  // 航向角误差（使用前视点，与PID控制器一致）
  double heading_error = compute_heading_error(
      vehicle_state, reference_state_lookahead);
  
  // 速度误差
  double speed_error = reference_state_lookahead[3] - vehicle_state.speed_;
  
  // 使用LQR反馈控制律（类似PID控制器的方式）
  // 横向控制：组合横向误差和航向角误差（与PID控制器一致）
  double lateral_control_error = heading_error + 0.5 * lateral_error;
  angular_velocity = K_[0][0] * lateral_error +        // 横向位置误差
                     K_[0][1] * lateral_control_error; // 组合误差（主要，类似PID的kp*error）
  
  // 纵向控制：速度跟踪（与PID控制器一致）
  acceleration = K_[1][3] * speed_error;                // 速度误差（主要）
  
  // 限制输出
  angular_velocity = std::clamp(angular_velocity, 
                                min_angular_velocity_, 
                                max_angular_velocity_);
  acceleration = std::clamp(acceleration, 
                           min_acceleration_, 
                           max_acceleration_);
}

void LQRController::reset() {
  // LQR控制器重置（如果需要）
  gain_computed_ = false;
  compute_lqr_gain();
}

void LQRController::compute_lqr_gain() {
  // LQR增益矩阵设计（简化版，更接近PID控制器）
  // 状态向量: [lateral_error, lateral_control_error, speed_error]
  // 控制向量: [angular_velocity, acceleration]
  // K = [k11, k12]  // 角速度控制
  //     [k21, k22]  // 加速度控制
  
  // 横向控制增益（角速度）
  // 基于权重矩阵Q、R，参考PID控制器的kp值范围
  double k_lateral = std::sqrt(q_y_ / r_angular_) * 0.3;      // 横向位置反馈增益
  double k_combined = std::sqrt(q_theta_ / r_angular_) * 1.2; // 组合误差反馈增益（主要）
  
  // 纵向控制增益（加速度）
  // 基于权重矩阵Q、R，参考PID控制器的kp值范围
  double k_speed = std::sqrt(q_v_ / r_acceleration_) * 1.8;    // 速度反馈增益
  
  // 设置增益矩阵（2x2简化版）
  // 第一行：角速度控制 [lateral_error, lateral_control_error]
  K_[0][0] = k_lateral;     // 横向误差 -> angular_velocity
  K_[0][1] = k_combined;    // 组合误差 -> angular_velocity（主要）
  K_[0][2] = 0.0;           // 未使用
  K_[0][3] = 0.0;           // 未使用
  
  // 第二行：加速度控制 [speed_error]
  K_[1][0] = 0.0;           // 未使用
  K_[1][1] = 0.0;           // 未使用
  K_[1][2] = 0.0;           // 未使用
  K_[1][3] = k_speed;       // 速度误差 -> acceleration（主要）
  
  gain_computed_ = true;
}

std::pair<std::vector<std::vector<double>>, 
          std::vector<std::vector<double>>> 
LQRController::linearize_vehicle_model(
    const std::vector<double>& state,
    const std::vector<double>& control) {
  
  // 车辆运动学模型线性化
  // state: [x, y, theta, v]
  // control: [angular_velocity, acceleration]
  
  double theta = state[2];
  double v = state[3];
  double dt = config_.dt_;
  
  // 状态矩阵 A (4x4)
  std::vector<std::vector<double>> A(4, std::vector<double>(4, 0.0));
  A[0][0] = 1.0;                    // dx/dx
  A[0][2] = -v * std::sin(theta) * dt;  // dx/dtheta
  A[0][3] = std::cos(theta) * dt;  // dx/dv
  
  A[1][1] = 1.0;                    // dy/dy
  A[1][2] = v * std::cos(theta) * dt;   // dy/dtheta
  A[1][3] = std::sin(theta) * dt;  // dy/dv
  
  A[2][2] = 1.0;                    // dtheta/dtheta
  
  A[3][3] = 1.0;                    // dv/dv
  
  // 控制矩阵 B (4x2)
  std::vector<std::vector<double>> B(4, std::vector<double>(2, 0.0));
  B[2][0] = dt;                     // dtheta/d(angular_velocity)
  B[3][1] = dt;                     // dv/d(acceleration)
  
  return std::make_pair(A, B);
}

double LQRController::compute_lateral_error(
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

double LQRController::compute_longitudinal_error(
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
  
  // 计算纵向误差（沿轨迹方向的距离）
  double cos_theta = std::cos(yaw);
  double sin_theta = std::sin(yaw);
  double longitudinal_error = dx * cos_theta + dy * sin_theta;
  
  return longitudinal_error;
}

double LQRController::compute_heading_error(
    const VehicleState& vehicle_state,
    const std::vector<double>& reference_state) {
  
  // 计算航向角误差（归一化到[-π, π]）
  // 注意：与PID控制器保持一致，使用 reference - current
  double heading_error = reference_state[2] - vehicle_state.theta_;
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
  
  return heading_error;
}

std::vector<double> LQRController::extract_reference_state(
    const LocalTrajectory::SharedPtr trajectory,
    int idx) {
  
  std::vector<double> reference_state(4);
  
  // 确保索引有效
  int trajectory_size = trajectory->local_trajectory.size();
  idx = std::min(idx, trajectory_size - 1);
  idx = std::max(idx, 0);
  
  const auto& point = trajectory->local_trajectory[idx].path_point;
  
  // 提取位置
  reference_state[0] = point.pose.pose.position.x;
  reference_state[1] = point.pose.pose.position.y;
  
  // 提取航向角
  tf2::Quaternion qtn(point.pose.pose.orientation.x,
                      point.pose.pose.orientation.y,
                      point.pose.pose.orientation.z,
                      point.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);
  reference_state[2] = yaw;
  
  // 提取速度（如果有，否则使用默认值）
  reference_state[3] = 10.0;  // 默认目标速度，可以从轨迹中获取
  
  return reference_state;
}

}  // namespace Control

