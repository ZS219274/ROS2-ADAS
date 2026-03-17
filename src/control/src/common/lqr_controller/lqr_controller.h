#ifndef LQR_CONTROLLER_H_
#define LQR_CONTROLLER_H_

#include <array>
#include "../controller_base/controller_base.h"
#include "../config_reader/config_reader.h"

namespace Control {

/**
 * @brief 基于离散代数Riccati方程（DARE）的LQR轨迹跟踪控制器
 *
 * 横向控制：自行车运动学模型，状态 [e_d, e_φ]，控制量 δ（转向角）
 *   - e_d: 横向位置误差（Frenet坐标系，正值表示车辆在轨迹左侧）
 *   - e_φ: 航向角误差（θ_vehicle − θ_ref，正值表示车辆偏左）
 *   - 误差动力学（连续）: ė_d = v·e_φ,  ė_φ = v·δ/L（前馈消除曲率后）
 *   - 曲率前馈: δ_ff = κ·L
 *   - 输出角速度: ω = v·tan(δ_fb + δ_ff) / L
 *
 * 纵向控制：质点运动模型，状态 [e_s, e_v]，控制量 a
 *   - e_s: 纵向位置误差（s_vehicle − s_ref）
 *   - e_v: 速度误差（v_vehicle − v_ref）
 *   - 参考加速度前馈: a = −K·[e_s, e_v]ᵀ + a_ref
 *
 * 横向增益随车速实时更新（A矩阵依赖速度v），纵向增益速度无关仅计算一次。
 */
class LQRController : public ControllerBase {
 public:
  explicit LQRController(const ControlConfigStruct& config);

  void compute_control_inputs(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int closest_idx, int lookahead_idx,
      double& angular_velocity, double& acceleration) override;

  void reset() override;

 private:
  using Mat2 = std::array<double, 4>;  // row-major 2×2: [a00, a01, a10, a11]
  using Vec2 = std::array<double, 2>;

  static Mat2 mat_add(const Mat2& a, const Mat2& b);
  static Mat2 mat_sub(const Mat2& a, const Mat2& b);
  static Mat2 mat_mul(const Mat2& a, const Mat2& b);
  static Mat2 mat_transpose(const Mat2& a);
  static Vec2 mat_vec_mul(const Mat2& m, const Vec2& v);
  static double normalize_angle(double angle);

  /**
   * @brief 求解 2-state / 1-input 离散代数Riccati方程
   * P = Aᵀ P A − Aᵀ P B (R + Bᵀ P B)⁻¹ Bᵀ P A + Q
   * @return 最优反馈增益 K (1×2)，使 u = −K·x 最小化 J = Σ(xᵀQx + uᵀRu)
   */
  Vec2 solve_dare(const Mat2& A, const Vec2& B,
                  const Mat2& Q, double R) const;

  Vec2 compute_lateral_gain(double v) const;
  Vec2 compute_longitudinal_gain() const;

  double compute_lateral_error(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state, int idx) const;

  double compute_heading_error(double vehicle_theta,
                               double ref_theta) const;

  double compute_station_error(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state, int idx) const;

  double extract_ref_yaw(
      const LocalTrajectory::SharedPtr trajectory, int idx) const;
  double extract_ref_speed(
      const LocalTrajectory::SharedPtr trajectory, int idx) const;
  double extract_ref_curvature(
      const LocalTrajectory::SharedPtr trajectory, int idx) const;
  double extract_ref_acceleration(
      const LocalTrajectory::SharedPtr trajectory, int idx) const;

  ControlConfigStruct config_;

  Vec2 K_lat_;                  // 横向反馈增益 [k_ed, k_ephi]
  Vec2 K_lon_;                  // 纵向反馈增益 [k_es, k_ev]
  double cached_lat_velocity_;  // 上次计算横向增益时的速度

  double max_angular_velocity_;
  double min_angular_velocity_;
  double max_acceleration_;
  double min_acceleration_;

  static constexpr double kMinComputeVelocity = 0.5;     // 增益计算的最小速度阈值
  static constexpr double kVelocityUpdateThreshold = 0.5; // 触发增益重算的速度变化量
  static constexpr double kMaxSteeringAngle = 1.0;        // 转向角限幅 (rad)
};

}  // namespace Control

#endif  // LQR_CONTROLLER_H_
