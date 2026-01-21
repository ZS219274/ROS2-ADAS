#include "mpc_controller.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace Control {

MPCController::MPCController(const ControlConfigStruct& config)
    : config_(config) {
  // 初始化MPC参数
  prediction_horizon_ = 10;      // 预测时域长度
  control_horizon_ = 5;          // 控制时域长度
  q_position_ = 1.0;             // 位置权重
  q_heading_ = 2.0;              // 航向角权重
  q_velocity_ = 1.0;             // 速度权重
  r_angular_ = 0.1;              // 角速度权重
  r_acceleration_ = 0.1;         // 加速度权重

  // 设置控制输入限制
  max_angular_velocity_ = config_.lateral_.max_output_;
  min_angular_velocity_ = config_.lateral_.min_output_;
  max_acceleration_ = config_.longitudinal_.max_output_;
  min_acceleration_ = config_.longitudinal_.min_output_;
}

void MPCController::compute_control_inputs(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state,
    int closest_idx, int lookahead_idx,
    double& angular_velocity, double& acceleration) {
  
  // 使用MPC优化计算控制输入
  std::vector<double> optimal_control = optimize_control(
      vehicle_state, trajectory, closest_idx, lookahead_idx);
  
  angular_velocity = optimal_control[0];
  acceleration = optimal_control[1];
  
  // 限制输出
  angular_velocity = std::clamp(angular_velocity, 
                                min_angular_velocity_, 
                                max_angular_velocity_);
  acceleration = std::clamp(acceleration, 
                           min_acceleration_, 
                           max_acceleration_);
}

void MPCController::reset() {
  // MPC控制器重置（如果需要）
}

std::vector<double> MPCController::vehicle_model(
    const std::vector<double>& state,
    const std::vector<double>& control,
    double dt) {
  // 车辆运动学模型
  // state: [x, y, theta, v]
  // control: [angular_velocity, acceleration]
  
  double x = state[0];
  double y = state[1];
  double theta = state[2];
  double v = state[3];
  
  double angular_vel = control[0];
  double accel = control[1];
  
  // 更新状态
  double v_new = v + accel * dt;
  v_new = std::max(0.0, v_new);  // 速度不能为负
  
  double theta_new = theta + angular_vel * dt;
  // 归一化航向角
  while (theta_new > M_PI) theta_new -= 2.0 * M_PI;
  while (theta_new < -M_PI) theta_new += 2.0 * M_PI;
  
  double x_new = x + v_new * std::cos(theta_new) * dt;
  double y_new = y + v_new * std::sin(theta_new) * dt;
  
  return {x_new, y_new, theta_new, v_new};
}

double MPCController::compute_cost(
    const std::vector<std::vector<double>>& predicted_states,
    const std::vector<std::vector<double>>& reference_states,
    const std::vector<std::vector<double>>& control_inputs) {
  
  double cost = 0.0;
  
  // 状态误差代价
  for (size_t i = 0; i < predicted_states.size(); ++i) {
    double dx = predicted_states[i][0] - reference_states[i][0];
    double dy = predicted_states[i][1] - reference_states[i][1];
    double dtheta = predicted_states[i][2] - reference_states[i][2];
    double dv = predicted_states[i][3] - reference_states[i][3];
    
    // 归一化航向角误差
    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    
    cost += q_position_ * (dx * dx + dy * dy);
    cost += q_heading_ * dtheta * dtheta;
    cost += q_velocity_ * dv * dv;
  }
  
  // 控制输入代价
  for (size_t i = 0; i < control_inputs.size(); ++i) {
    cost += r_angular_ * control_inputs[i][0] * control_inputs[i][0];
    cost += r_acceleration_ * control_inputs[i][1] * control_inputs[i][1];
  }
  
  return cost;
}

std::vector<double> MPCController::optimize_control(
    const VehicleState& vehicle_state,
    const LocalTrajectory::SharedPtr trajectory,
    int closest_idx, int lookahead_idx) {
  
  // 构建参考轨迹
  std::vector<std::vector<double>> reference_states;
  int trajectory_size = trajectory->local_trajectory.size();
  
  for (int i = 0; i < prediction_horizon_; ++i) {
    int idx = std::min(closest_idx + i, trajectory_size - 1);
    const auto& point = trajectory->local_trajectory[idx].path_point;
    
    // 提取参考状态
    double ref_x = point.pose.pose.position.x;
    double ref_y = point.pose.pose.position.y;
    
    // 提取航向角
    tf2::Quaternion qtn(point.pose.pose.orientation.x,
                        point.pose.pose.orientation.y,
                        point.pose.pose.orientation.z,
                        point.pose.pose.orientation.w);
    double roll, pitch, ref_theta;
    tf2::Matrix3x3(qtn).getRPY(roll, pitch, ref_theta);
    
    // 提取速度（如果有）
    double ref_v = 10.0;  // 默认目标速度，可以从轨迹中获取
    
    reference_states.push_back({ref_x, ref_y, ref_theta, ref_v});
  }
  
  // 当前状态
  std::vector<double> current_state = {
      vehicle_state.pose_x_,
      vehicle_state.pose_y_,
      vehicle_state.theta_,
      vehicle_state.speed_
  };
  
  // 简化的MPC优化：使用梯度下降法
  // 在实际应用中，可以使用更高效的优化算法（如QP求解器）
  double best_cost = std::numeric_limits<double>::max();
  std::vector<double> best_control = {0.0, 0.0};
  
  // 搜索空间：角速度和加速度
  int search_steps = 20;
  double angular_range = max_angular_velocity_ - min_angular_velocity_;
  double accel_range = max_acceleration_ - min_acceleration_;
  
  for (int i = 0; i < search_steps; ++i) {
    for (int j = 0; j < search_steps; ++j) {
      double angular_vel = min_angular_velocity_ + 
                          (angular_range * i / (search_steps - 1));
      double accel = min_acceleration_ + 
                    (accel_range * j / (search_steps - 1));
      
      // 预测未来状态
      std::vector<std::vector<double>> predicted_states;
      std::vector<double> state = current_state;
      
      std::vector<std::vector<double>> control_inputs;
      for (int k = 0; k < prediction_horizon_; ++k) {
        // 使用前control_horizon_个控制输入，之后保持最后一个
        std::vector<double> control;
        if (k < control_horizon_) {
          control = {angular_vel, accel};
        } else {
          control = control_inputs.back();
        }
        control_inputs.push_back(control);
        
        state = vehicle_model(state, control, config_.dt_);
        predicted_states.push_back(state);
      }
      
      // 计算代价
      double cost = compute_cost(predicted_states, reference_states, 
                                control_inputs);
      
      if (cost < best_cost) {
        best_cost = cost;
        best_control = {angular_vel, accel};
      }
    }
  }
  
  return best_control;
}

}  // namespace Control

