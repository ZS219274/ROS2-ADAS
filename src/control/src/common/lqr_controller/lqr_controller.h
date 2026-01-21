#ifndef LQR_CONTROLLER_H_
#define LQR_CONTROLLER_H_

#include <vector>
#include <memory>
#include "../controller_base/controller_base.h"
#include "../config_reader/config_reader.h"

namespace Control {

/**
 * @brief LQR控制器类
 * 
 * 实现线性二次调节器（Linear Quadratic Regulator）算法
 * 用于车辆轨迹跟踪控制
 */
class LQRController : public ControllerBase {
 public:
  /**
   * @brief 构造函数
   * @param config 控制配置
   */
  explicit LQRController(const ControlConfigStruct& config);

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
   * @brief 计算LQR反馈增益矩阵
   * 使用Riccati方程求解最优反馈增益
   */
  void compute_lqr_gain();

  /**
   * @brief 线性化车辆模型
   * @param state 当前状态 [x, y, theta, v]
   * @param control 控制输入 [angular_velocity, acceleration]
   * @return 状态矩阵A和控制矩阵B
   */
  std::pair<std::vector<std::vector<double>>, 
            std::vector<std::vector<double>>> 
  linearize_vehicle_model(const std::vector<double>& state,
                          const std::vector<double>& control);

  /**
   * @brief 计算横向误差（垂直于轨迹方向的距离）
   */
  double compute_lateral_error(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int closest_idx);

  /**
   * @brief 计算纵向误差（沿轨迹方向的距离）
   */
  double compute_longitudinal_error(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int closest_idx);

  /**
   * @brief 计算航向角误差
   */
  double compute_heading_error(
      const VehicleState& vehicle_state,
      const std::vector<double>& reference_state);

  /**
   * @brief 从轨迹中提取参考状态
   * @param trajectory 轨迹
   * @param idx 轨迹点索引
   * @return 参考状态 [x, y, theta, v]
   */
  std::vector<double> extract_reference_state(
      const LocalTrajectory::SharedPtr trajectory,
      int idx);

  ControlConfigStruct config_;  // 控制配置

  // LQR参数
  std::vector<std::vector<double>> K_;  // LQR反馈增益矩阵 [2x4]
  
  // 权重矩阵 Q (状态权重) 和 R (控制权重)
  double q_x_;              // x位置权重
  double q_y_;              // y位置权重
  double q_theta_;          // 航向角权重
  double q_v_;              // 速度权重
  double r_angular_;        // 角速度权重
  double r_acceleration_;   // 加速度权重

  // 控制输入限制
  double max_angular_velocity_;  // 最大角速度
  double min_angular_velocity_; // 最小角速度
  double max_acceleration_;      // 最大加速度
  double min_acceleration_;      // 最小加速度

  bool gain_computed_;  // 增益矩阵是否已计算
};

}  // namespace Control

#endif  // LQR_CONTROLLER_H_

