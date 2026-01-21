#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include "../controller_base/controller_base.h"
#include "../config_reader/config_reader.h"
#include "base_msgs/msg/local_trajectory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace Control {
/**
 * @brief PID控制器类
 * 
 * 实现标准的PID控制算法：
 * u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
 */
class PIDController {
 public:
  /**
   * @brief 构造函数
   * @param kp 比例系数
   * @param ki 积分系数
   * @param kd 微分系数
   * @param dt 控制周期（秒）
   * @param max_output 最大输出限制
   * @param min_output 最小输出限制
   */
  PIDController(double kp, double ki, double kd, double dt,
                double max_output = 1.0, double min_output = -1.0);

  /**
   * @brief 计算控制输出
   * @param error 当前误差
   * @return 控制输出
   */
  double compute(double error);

  /**
   * @brief 重置控制器（清除积分项和上次误差）
   */
  void reset();

  /**
   * @brief 设置PID参数
   */
  void setParameters(double kp, double ki, double kd);

  /**
   * @brief 设置输出限制
   */
  void setOutputLimits(double min_output, double max_output);

 private:
  double kp_;           // 比例系数
  double ki_;           // 积分系数
  double kd_;           // 微分系数
  double dt_;           // 控制周期
  double max_output_;   // 最大输出
  double min_output_;   // 最小输出

  double integral_;     // 积分项累积
  double prev_error_;   // 上次误差（用于计算微分项）
  bool first_run_;      // 是否是第一次运行
};

/**
 * @brief PID控制器包装类
 * 
 * 将PID控制器包装成ControllerBase接口，使其可以与MPC控制器统一使用
 */
class PIDControllerWrapper : public ControllerBase {
 public:
  /**
   * @brief 构造函数
   * @param config 控制配置
   */
  explicit PIDControllerWrapper(const ControlConfigStruct& config);

  /**
   * @brief 计算控制输出
   */
  void compute_control_inputs(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int closest_idx, int lookahead_idx,
      double& angular_velocity, double& acceleration) override;

  /**
   * @brief 重置控制器
   */
  void reset() override;

 private:
  /**
   * @brief 计算横向误差
   */
  double compute_cross_track_error(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int closest_idx);

  /**
   * @brief 计算航向角误差
   */
  double compute_heading_error(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int lookahead_idx);

  std::unique_ptr<PIDController> lateral_pid_;      // 横向控制PID
  std::unique_ptr<PIDController> longitudinal_pid_; // 纵向控制PID
  double target_speed_;                              // 目标速度
};

}  // namespace Control

#endif  // PID_CONTROLLER_H_

