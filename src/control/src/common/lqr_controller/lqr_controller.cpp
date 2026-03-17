#include "lqr_controller.h"
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace Control {

// =====================================================================
//  2×2 矩阵运算（row-major: [a00, a01, a10, a11]）
// =====================================================================

LQRController::Mat2 LQRController::mat_add(const Mat2& a, const Mat2& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
}

LQRController::Mat2 LQRController::mat_sub(const Mat2& a, const Mat2& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]};
}

LQRController::Mat2 LQRController::mat_mul(const Mat2& a, const Mat2& b) {
  return {a[0] * b[0] + a[1] * b[2], a[0] * b[1] + a[1] * b[3],
          a[2] * b[0] + a[3] * b[2], a[2] * b[1] + a[3] * b[3]};
}

LQRController::Mat2 LQRController::mat_transpose(const Mat2& a) {
  return {a[0], a[2], a[1], a[3]};
}

LQRController::Vec2 LQRController::mat_vec_mul(const Mat2& m,
                                                const Vec2& v) {
  return {m[0] * v[0] + m[1] * v[1], m[2] * v[0] + m[3] * v[1]};
}

double LQRController::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// =====================================================================
//  DARE 求解器  —  2-state / 1-input 离散代数Riccati方程
// =====================================================================

LQRController::Vec2 LQRController::solve_dare(const Mat2& A, const Vec2& B,
                                               const Mat2& Q,
                                               double R) const {
  const int max_iter = config_.lqr_.max_iteration;
  const double tol = config_.lqr_.tolerance;
  Mat2 P = Q;
  Mat2 At = mat_transpose(A);

  for (int iter = 0; iter < max_iter; ++iter) {
    Mat2 P_prev = P;

    // Aᵀ P,  Aᵀ P A
    Mat2 AtP = mat_mul(At, P);
    Mat2 AtPA = mat_mul(AtP, A);

    // Aᵀ P B  (2×1)
    Vec2 AtPB = mat_vec_mul(AtP, B);

    // Bᵀ P B  (scalar)
    Vec2 PB = mat_vec_mul(P, B);
    double BtPB = B[0] * PB[0] + B[1] * PB[1];
    double inv_factor = 1.0 / (R + BtPB);

    // rank-1 修正: Aᵀ P B · (R + Bᵀ P B)⁻¹ · Bᵀ P A
    Mat2 correction = {
        AtPB[0] * inv_factor * AtPB[0], AtPB[0] * inv_factor * AtPB[1],
        AtPB[1] * inv_factor * AtPB[0], AtPB[1] * inv_factor * AtPB[1]};

    P = mat_add(mat_sub(AtPA, correction), Q);

    double max_diff =
        std::max({std::abs(P[0] - P_prev[0]), std::abs(P[1] - P_prev[1]),
                  std::abs(P[2] - P_prev[2]), std::abs(P[3] - P_prev[3])});
    if (max_diff < tol) {
      RCLCPP_DEBUG(rclcpp::get_logger("lqr_controller"),
                   "DARE converged in %d iterations (diff=%.2e)", iter + 1,
                   max_diff);
      break;
    }
  }

  // K = (R + Bᵀ P B)⁻¹ · Bᵀ P A   (1×2)
  Vec2 PB = mat_vec_mul(P, B);
  double BtPB = B[0] * PB[0] + B[1] * PB[1];
  double inv_factor = 1.0 / (R + BtPB);

  double BtP0 = B[0] * P[0] + B[1] * P[2];
  double BtP1 = B[0] * P[1] + B[1] * P[3];

  double k0 = inv_factor * (BtP0 * A[0] + BtP1 * A[2]);
  double k1 = inv_factor * (BtP0 * A[1] + BtP1 * A[3]);

  return {k0, k1};
}

// =====================================================================
//  增益计算
// =====================================================================

LQRController::Vec2 LQRController::compute_lateral_gain(double v) const {
  double dt = config_.dt_;
  double L = config_.wheelbase_;

  // 自行车运动学横向误差模型（曲率前馈消除后）
  //   x = [e_d, e_φ],  u = δ
  //   A = [[1, v·dt], [0, 1]],  B = [[0], [v·dt/L]]
  Mat2 A = {1.0, v * dt, 0.0, 1.0};
  Vec2 B = {0.0, v * dt / L};

  Mat2 Q = {config_.lqr_.q_lateral_error, 0.0, 0.0,
            config_.lqr_.q_heading_error};
  double R = config_.lqr_.r_steering;

  return solve_dare(A, B, Q, R);
}

LQRController::Vec2 LQRController::compute_longitudinal_gain() const {
  double dt = config_.dt_;

  // 纵向误差模型
  //   x = [e_s, e_v],  u = a − a_ref
  //   A = [[1, dt], [0, 1]],  B = [[0], [dt]]
  Mat2 A = {1.0, dt, 0.0, 1.0};
  Vec2 B = {0.0, dt};

  Mat2 Q = {config_.lqr_.q_station_error, 0.0, 0.0,
            config_.lqr_.q_velocity_error};
  double R = config_.lqr_.r_acceleration;

  return solve_dare(A, B, Q, R);
}

// =====================================================================
//  构造 / 重置
// =====================================================================

LQRController::LQRController(const ControlConfigStruct& config)
    : config_(config), cached_lat_velocity_(-1.0) {
  max_angular_velocity_ = config_.lateral_.max_output_;
  min_angular_velocity_ = config_.lateral_.min_output_;
  max_acceleration_ = config_.longitudinal_.max_output_;
  min_acceleration_ = config_.longitudinal_.min_output_;

  K_lon_ = compute_longitudinal_gain();
  K_lat_ = {0.0, 0.0};

  RCLCPP_INFO(rclcpp::get_logger("lqr_controller"),
              "LQR controller initialized (DARE-based). "
              "K_lon=[%.4f, %.4f], wheelbase=%.2f, dt=%.3f",
              K_lon_[0], K_lon_[1], config_.wheelbase_, config_.dt_);
}

void LQRController::reset() {
  cached_lat_velocity_ = -1.0;
  K_lat_ = {0.0, 0.0};
  K_lon_ = compute_longitudinal_gain();
}

// =====================================================================
//  主控制回路
// =====================================================================

void LQRController::compute_control_inputs(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state, int closest_idx,
    int /*lookahead_idx*/, double& angular_velocity, double& acceleration) {
  double v = vehicle_state.speed_;
  double L = config_.wheelbase_;
  double v_ctrl = std::max(std::abs(v), kMinComputeVelocity);

  // ---- 横向增益：随速度自适应更新 ----
  if (cached_lat_velocity_ < 0.0 ||
      std::abs(v_ctrl - cached_lat_velocity_) > kVelocityUpdateThreshold) {
    K_lat_ = compute_lateral_gain(v_ctrl);
    cached_lat_velocity_ = v_ctrl;
    RCLCPP_DEBUG(rclcpp::get_logger("lqr_controller"),
                 "Lateral gain updated at v=%.2f: K=[%.4f, %.4f]", v_ctrl,
                 K_lat_[0], K_lat_[1]);
  }

  // =============== 横向控制 ===============
  double ref_yaw = extract_ref_yaw(trajectory, closest_idx);
  double ref_kappa = extract_ref_curvature(trajectory, closest_idx);

  double e_d = compute_lateral_error(trajectory, vehicle_state, closest_idx);
  double e_phi = compute_heading_error(vehicle_state.theta_, ref_yaw);

  // LQR反馈: δ_fb = −K · [e_d, e_φ]ᵀ
  double delta_fb = -(K_lat_[0] * e_d + K_lat_[1] * e_phi);

  // 曲率前馈: δ_ff = κ · L
  double delta_ff = ref_kappa * L;

  double delta =
      std::clamp(delta_fb + delta_ff, -kMaxSteeringAngle, kMaxSteeringAngle);

  // 转向角 → 角速度: ω = v · tan(δ) / L
  angular_velocity = v_ctrl * std::tan(delta) / L;

  // =============== 纵向控制 ===============
  double ref_speed = extract_ref_speed(trajectory, closest_idx);
  double ref_accel = extract_ref_acceleration(trajectory, closest_idx);

  // e_s = s_vehicle − s_ref (正值：车辆超前)
  double e_s = -compute_station_error(trajectory, vehicle_state, closest_idx);
  // e_v = v_vehicle − v_ref (正值：车辆过快)
  double e_v = v - ref_speed;

  // LQR反馈 + 参考加速度前馈: a = −K · [e_s, e_v]ᵀ + a_ref
  acceleration = -(K_lon_[0] * e_s + K_lon_[1] * e_v) + ref_accel;

  // =============== 输出限幅 ===============
  angular_velocity =
      std::clamp(angular_velocity, min_angular_velocity_, max_angular_velocity_);
  acceleration =
      std::clamp(acceleration, min_acceleration_, max_acceleration_);
}

// =====================================================================
//  误差计算
// =====================================================================

double LQRController::compute_lateral_error(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state, int idx) const {
  const auto& pt = trajectory->local_trajectory[idx].path_point;

  // 车辆相对轨迹点的偏移
  double dx = vehicle_state.pose_x_ - pt.pose.pose.position.x;
  double dy = vehicle_state.pose_y_ - pt.pose.pose.position.y;
  double yaw = pt.theta;

  // 投影到轨迹左法向 n_left = (−sin θ, cos θ)
  // 正值 ⟹ 车辆在轨迹左侧
  return -dx * std::sin(yaw) + dy * std::cos(yaw);
}

double LQRController::compute_heading_error(double vehicle_theta,
                                            double ref_theta) const {
  // e_φ = θ_vehicle − θ_ref,  归一化到 [−π, π]
  return normalize_angle(vehicle_theta - ref_theta);
}

double LQRController::compute_station_error(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state, int idx) const {
  const auto& pt = trajectory->local_trajectory[idx].path_point;

  double dx = pt.pose.pose.position.x - vehicle_state.pose_x_;
  double dy = pt.pose.pose.position.y - vehicle_state.pose_y_;
  double yaw = pt.theta;

  // 轨迹点相对车辆的纵向分量，正值表示轨迹点在车辆前方（车辆落后）
  return dx * std::cos(yaw) + dy * std::sin(yaw);
}

// =====================================================================
//  参考量提取
// =====================================================================

double LQRController::extract_ref_yaw(
    const LocalTrajectory::SharedPtr trajectory, int idx) const {
  int size = static_cast<int>(trajectory->local_trajectory.size());
  idx = std::clamp(idx, 0, size - 1);
  return trajectory->local_trajectory[idx].path_point.theta;
}

double LQRController::extract_ref_speed(
    const LocalTrajectory::SharedPtr trajectory, int idx) const {
  int size = static_cast<int>(trajectory->local_trajectory.size());
  idx = std::clamp(idx, 0, size - 1);
  return trajectory->local_trajectory[idx].speed_point.speed;
}

double LQRController::extract_ref_curvature(
    const LocalTrajectory::SharedPtr trajectory, int idx) const {
  int size = static_cast<int>(trajectory->local_trajectory.size());
  idx = std::clamp(idx, 0, size - 1);
  return trajectory->local_trajectory[idx].path_point.kappa;
}

double LQRController::extract_ref_acceleration(
    const LocalTrajectory::SharedPtr trajectory, int idx) const {
  int size = static_cast<int>(trajectory->local_trajectory.size());
  idx = std::clamp(idx, 0, size - 1);
  return trajectory->local_trajectory[idx].speed_point.acceleration;
}

}  // namespace Control
