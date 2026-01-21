#ifndef CONTROLLER_BASE_H_
#define CONTROLLER_BASE_H_

#include "base_msgs/msg/local_trajectory.hpp"

namespace Control {
using base_msgs::msg::LocalTrajectory;

/**
 * @brief 控制类型枚举
 */
enum class ControlType {
  KPID = 0,   // PID控制器
  KMPC = 1,   // MPC控制器
  KLQR = 2    // LQR控制器
};

/**
 * @brief 控制器基类
 * 
 * 定义统一的控制器接口，支持不同控制算法
 */
class ControllerBase {
 public:
  virtual ~ControllerBase() {}

  /**
   * @brief 计算控制输出
   * @param trajectory 局部轨迹
   * @param vehicle_state 车辆当前状态
   * @param closest_idx 最近轨迹点索引
   * @param lookahead_idx 前视点索引
   * @param angular_velocity 输出的角速度（rad/s）
   * @param acceleration 输出的加速度（m/s²）
   */
  virtual void compute_control_inputs(
      const LocalTrajectory::SharedPtr trajectory,
      const struct VehicleState& vehicle_state,
      int closest_idx, int lookahead_idx,
      double& angular_velocity, double& acceleration) = 0;

  /**
   * @brief 重置控制器
   */
  virtual void reset() = 0;
};

/**
 * @brief 车辆状态结构体
 */
struct VehicleState {
  double pose_x_ = 0.0;   // x坐标
  double pose_y_ = 0.0;   // y坐标
  double theta_ = 0.0;    // 航向角
  double speed_ = 0.0;    // 速度
};

}  // namespace Control

#endif  // CONTROLLER_BASE_H_

