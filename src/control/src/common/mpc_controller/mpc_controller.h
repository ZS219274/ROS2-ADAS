#ifndef MPC_CONTROLLER_H_
#define MPC_CONTROLLER_H_

#include <vector>
#include <memory>
#include "../controller_base/controller_base.h"
#include "../config_reader/config_reader.h"

namespace Control {

/**
 * @brief MPC控制器类
 * 
 * 实现模型预测控制（Model Predictive Control）算法
 * 用于车辆轨迹跟踪控制
 */
class MPCController : public ControllerBase {
 public:
  /**
   * @brief 构造函数
   * @param config 控制配置
   */
  explicit MPCController(const ControlConfigStruct& config);

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
   * @brief 车辆运动学模型
   * @param state 当前状态 [x, y, theta, v]
   * @param control 控制输入 [angular_velocity, acceleration]
   * @param dt 时间步长
   * @return 下一时刻状态
   */
  std::vector<double> vehicle_model(const std::vector<double>& state,
                                    const std::vector<double>& control,
                                    double dt);

  /**
   * @brief 计算代价函数
   * @param predicted_states 预测状态序列
   * @param reference_states 参考状态序列
   * @param control_inputs 控制输入序列
   * @return 代价函数值
   */
  double compute_cost(const std::vector<std::vector<double>>& predicted_states,
                      const std::vector<std::vector<double>>& reference_states,
                      const std::vector<std::vector<double>>& control_inputs);

  /**
   * @brief 优化控制输入
   * @param vehicle_state 当前车辆状态
   * @param trajectory 参考轨迹
   * @param closest_idx 最近点索引
   * @param lookahead_idx 前视点索引
   * @return 最优控制输入 [angular_velocity, acceleration]
   */
  std::vector<double> optimize_control(
      const VehicleState& vehicle_state,
      const LocalTrajectory::SharedPtr trajectory,
      int closest_idx, int lookahead_idx);

  ControlConfigStruct config_;  // 控制配置

  // MPC参数
  int prediction_horizon_;       // 预测时域长度
  int control_horizon_;          // 控制时域长度
  double q_position_;            // 位置权重
  double q_heading_;             // 航向角权重
  double q_velocity_;            // 速度权重
  double r_angular_;             // 角速度权重
  double r_acceleration_;        // 加速度权重

  // 控制输入限制
  double max_angular_velocity_;  // 最大角速度
  double min_angular_velocity_;  // 最小角速度
  double max_acceleration_;      // 最大加速度
  double min_acceleration_;      // 最小加速度
};

}  // namespace Control

#endif  // MPC_CONTROLLER_H_

