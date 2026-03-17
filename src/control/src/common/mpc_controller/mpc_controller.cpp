#include "mpc_controller.h"
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace Control {

// =====================================================================
//  Mat — 密集矩阵基础运算（列优先）
// =====================================================================

MPCController::Mat MPCController::Mat::operator+(const Mat& o) const {
  Mat res(r, c);
  for (size_t i = 0; i < d.size(); ++i) res.d[i] = d[i] + o.d[i];
  return res;
}

MPCController::Mat MPCController::Mat::operator-(const Mat& o) const {
  Mat res(r, c);
  for (size_t i = 0; i < d.size(); ++i) res.d[i] = d[i] - o.d[i];
  return res;
}

MPCController::Mat MPCController::Mat::operator*(const Mat& o) const {
  Mat res(r, o.c);
  for (int j = 0; j < o.c; ++j)
    for (int k = 0; k < c; ++k) {
      double bkj = o(k, j);
      for (int i = 0; i < r; ++i)
        res(i, j) += (*this)(i, k) * bkj;
    }
  return res;
}

MPCController::Mat MPCController::Mat::operator*(double s) const {
  Mat res(r, c);
  for (size_t i = 0; i < d.size(); ++i) res.d[i] = d[i] * s;
  return res;
}

MPCController::Mat MPCController::Mat::T() const {
  Mat res(c, r);
  for (int i = 0; i < r; ++i)
    for (int j = 0; j < c; ++j) res(j, i) = (*this)(i, j);
  return res;
}

void MPCController::Mat::set_block(int r0, int c0, const Mat& blk) {
  for (int j = 0; j < blk.c; ++j)
    for (int i = 0; i < blk.r; ++i)
      (*this)(r0 + i, c0 + j) = blk(i, j);
}

MPCController::Mat MPCController::Mat::block(int r0, int c0,
                                             int nr, int nc) const {
  Mat res(nr, nc);
  for (int j = 0; j < nc; ++j)
    for (int i = 0; i < nr; ++i) res(i, j) = (*this)(r0 + i, c0 + j);
  return res;
}

MPCController::Mat MPCController::Mat::eye(int n) {
  Mat res(n, n);
  for (int i = 0; i < n; ++i) res(i, i) = 1.0;
  return res;
}

double MPCController::Mat::sqnorm() const {
  double s = 0;
  for (auto x : d) s += x * x;
  return s;
}

double MPCController::norm_angle(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

// =====================================================================
//  自行车运动学模型线性化（Euler 离散）
// =====================================================================
//
//  连续模型: ẋ = v cos θ,  ẏ = v sin θ,  θ̇ = v tan δ / L,  v̇ = a
//
//  A_d = I + dt · ∂f/∂x,  B_d = dt · ∂f/∂u

void MPCController::linearize(double theta, double v, double delta,
                               Mat& Ad, Mat& Bd) const {
  double dt = config_.dt_;
  double L = config_.wheelbase_;
  double ct = std::cos(theta), st = std::sin(theta);
  double td = std::tan(delta);
  double cd2 = std::cos(delta); cd2 *= cd2;

  Ad = Mat::eye(NX);
  Ad(0, 2) += dt * (-v * st);       // ∂ẋ/∂θ
  Ad(0, 3) += dt * ct;              // ∂ẋ/∂v
  Ad(1, 2) += dt * (v * ct);        // ∂ẏ/∂θ
  Ad(1, 3) += dt * st;              // ∂ẏ/∂v
  Ad(2, 3) += dt * td / L;          // ∂θ̇/∂v

  Bd = Mat(NX, NU);
  Bd(2, 0) = dt * v / (L * cd2);    // ∂θ̇/∂δ
  Bd(3, 1) = dt;                     // ∂v̇/∂a
}

// =====================================================================
//  构建批量预测矩阵
// =====================================================================
//
//  误差动力学: ξ(k+1) = A_k ξ(k) + B_k Δu(k)
//  批量形式:   Ξ = S_x · ξ₀ + S_u · ΔU
//
//  S_x 每行块: Φ_i = A_i · A_{i-1} · … · A_0
//  S_u 递推:   S_u(i,j) = A_i · S_u(i-1,j)            for j < i
//              S_u(i,i) = B_i                            for i < Nc
//              S_u(i,Nc-1) += B_i                        for i ≥ Nc（控制保持）

void MPCController::build_prediction(const std::vector<Mat>& Ad,
                                     const std::vector<Mat>& Bd,
                                     Mat& Sx, Mat& Su) const {
  int Np = config_.mpc_.prediction_horizon;
  int Nc = config_.mpc_.control_horizon;

  Sx = Mat(NX * Np, NX);
  Su = Mat(NX * Np, NU * Nc);

  // S_x
  Mat Phi = Mat::eye(NX);
  for (int i = 0; i < Np; ++i) {
    Phi = Ad[i] * Phi;
    Sx.set_block(i * NX, 0, Phi);
  }

  // S_u（递推构建）
  for (int i = 0; i < Np; ++i) {
    int jmax = std::min(i, Nc - 1);
    for (int j = jmax; j >= 0; --j) {
      if (j == i && i < Nc) {
        Su.set_block(i * NX, j * NU, Bd[i]);
      } else if (i > 0) {
        Mat prev = Su.block((i - 1) * NX, j * NU, NX, NU);
        Mat cur = Ad[i] * prev;
        if (i >= Nc && j == Nc - 1) cur = cur + Bd[i];
        Su.set_block(i * NX, j * NU, cur);
      }
    }
  }
}

// =====================================================================
//  幂迭代法估计最大特征值
// =====================================================================

double MPCController::spectral_radius(const Mat& H) const {
  int n = H.r;
  Mat v(n, 1);
  for (int i = 0; i < n; ++i) v(i, 0) = 1.0;

  double lambda = 1.0;
  for (int iter = 0; iter < 30; ++iter) {
    Mat Hv = H * v;
    double nrm = std::sqrt(Hv.sqnorm());
    if (nrm < 1e-15) break;
    v = Hv * (1.0 / nrm);
    Mat vHv = v.T() * (H * v);
    lambda = vHv(0, 0);
  }
  return std::max(lambda, 1e-6);
}

// =====================================================================
//  投影梯度下降求解 box-constrained QP
// =====================================================================
//
//  min  0.5 · ΔUᵀ H ΔU + gᵀ ΔU
//  s.t. lb ≤ ΔU ≤ ub
//
//  步长 α = 1 / λ_max(H)

MPCController::Mat MPCController::solve_box_qp(
    const Mat& H, const Mat& g,
    const Mat& lb, const Mat& ub,
    const Mat& init) const {

  int n = H.r;
  double alpha = 1.0 / spectral_radius(H);
  int max_iter = config_.mpc_.solver_max_iter;
  double tol_sq = config_.mpc_.solver_tolerance *
                  config_.mpc_.solver_tolerance;

  Mat U = init;
  for (int i = 0; i < n; ++i)
    U(i, 0) = std::clamp(U(i, 0), lb(i, 0), ub(i, 0));

  for (int iter = 0; iter < max_iter; ++iter) {
    Mat grad = H * U + g;
    Mat U_new(n, 1);
    for (int i = 0; i < n; ++i)
      U_new(i, 0) = std::clamp(U(i, 0) - alpha * grad(i, 0),
                                lb(i, 0), ub(i, 0));
    if ((U_new - U).sqnorm() < tol_sq) {
      U = U_new;
      break;
    }
    U = U_new;
  }
  return U;
}

// =====================================================================
//  构造 / 重置
// =====================================================================

MPCController::MPCController(const ControlConfigStruct& config)
    : config_(config), has_prev_(false), prev_delta_(0), prev_accel_(0) {
  max_omega_ = config_.lateral_.max_output_;
  min_omega_ = config_.lateral_.min_output_;
  max_accel_ = config_.longitudinal_.max_output_;
  min_accel_ = config_.longitudinal_.min_output_;

  int Nc = config_.mpc_.control_horizon;
  prev_U_abs_.resize(NU * Nc, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"),
              "LTV-MPC initialized: Np=%d, Nc=%d, dt=%.3f, L=%.2f",
              config_.mpc_.prediction_horizon,
              config_.mpc_.control_horizon,
              config_.dt_, config_.wheelbase_);
}

void MPCController::reset() {
  has_prev_ = false;
  prev_delta_ = 0;
  prev_accel_ = 0;
  std::fill(prev_U_abs_.begin(), prev_U_abs_.end(), 0.0);
}

// =====================================================================
//  主控制回路
// =====================================================================

void MPCController::compute_control_inputs(
    const LocalTrajectory::SharedPtr trajectory,
    const VehicleState& vehicle_state,
    int closest_idx, int /*lookahead_idx*/,
    double& angular_velocity, double& acceleration) {

  const int Np = config_.mpc_.prediction_horizon;
  const int Nc = config_.mpc_.control_horizon;
  const double L = config_.wheelbase_;
  const double v = vehicle_state.speed_;
  const int traj_sz = static_cast<int>(trajectory->local_trajectory.size());

  // ===== 1. 提取参考轨迹 =====
  std::vector<double> x_r(Np), y_r(Np), th_r(Np), v_r(Np);
  std::vector<double> kap_r(Np), a_r(Np);
  for (int i = 0; i < Np; ++i) {
    int idx = std::clamp(closest_idx + i, 0, traj_sz - 1);
    const auto& pt = trajectory->local_trajectory[idx].path_point;
    const auto& sp = trajectory->local_trajectory[idx].speed_point;
    x_r[i] = pt.pose.pose.position.x;
    y_r[i] = pt.pose.pose.position.y;
    th_r[i] = pt.theta;
    kap_r[i] = pt.kappa;
    v_r[i] = sp.speed;
    if (v_r[i] < 1e-3) v_r[i] = config_.default_speed_;
    a_r[i] = sp.acceleration;
  }

  // 参考控制: δ_ref = atan(κ·L), a_ref 直接取速度规划
  std::vector<double> delta_r(Np), accel_r(Np);
  for (int i = 0; i < Np; ++i) {
    delta_r[i] = std::atan(kap_r[i] * L);
    accel_r[i] = a_r[i];
  }

  // ===== 2. 初始误差状态 ξ₀ = x₀ − x_ref(0) =====
  Mat xi0(NX, 1);
  xi0(0, 0) = vehicle_state.pose_x_ - x_r[0];
  xi0(1, 0) = vehicle_state.pose_y_ - y_r[0];
  xi0(2, 0) = norm_angle(vehicle_state.theta_ - th_r[0]);
  xi0(3, 0) = v - v_r[0];

  // ===== 3. 沿参考线性化 =====
  std::vector<Mat> Ad_list(Np), Bd_list(Np);
  for (int i = 0; i < Np; ++i)
    linearize(th_r[i], std::max(v_r[i], 0.5), delta_r[i],
              Ad_list[i], Bd_list[i]);

  // ===== 4. 构建预测矩阵 =====
  Mat Sx, Su;
  build_prediction(Ad_list, Bd_list, Sx, Su);

  // ===== 5. 构建 QP =====
  const int nu_total = NU * Nc;
  const int nx_total = NX * Np;

  // --- drift 向量 d = S_x · ξ₀ ---
  Mat d_vec = Sx * xi0;
  for (int i = 0; i < Np; ++i)
    d_vec(i * NX + 2, 0) = norm_angle(d_vec(i * NX + 2, 0));

  // --- H_state = S_uᵀ Q̄ S_u (利用 Q̄ 对角结构) ---
  std::vector<double> q_w(nx_total);
  for (int i = 0; i < Np; ++i) {
    q_w[i * NX + 0] = config_.mpc_.q_x;
    q_w[i * NX + 1] = config_.mpc_.q_y;
    q_w[i * NX + 2] = config_.mpc_.q_theta;
    q_w[i * NX + 3] = config_.mpc_.q_v;
  }

  Mat H(nu_total, nu_total);
  Mat g_vec(nu_total, 1);

  // S_uᵀ Q̄ S_u  和  S_uᵀ Q̄ d  (对角 Q̄ 加速计算)
  for (int i = 0; i < nu_total; ++i) {
    for (int j = i; j < nu_total; ++j) {
      double val = 0;
      for (int k = 0; k < nx_total; ++k)
        val += q_w[k] * Su(k, i) * Su(k, j);
      H(i, j) = val;
      H(j, i) = val;
    }
    double gval = 0;
    for (int k = 0; k < nx_total; ++k)
      gval += q_w[k] * Su(k, i) * d_vec(k, 0);
    g_vec(i, 0) = gval;
  }

  // --- 加上 R̄（控制量惩罚） ---
  for (int i = 0; i < Nc; ++i) {
    H(i * NU, i * NU) += config_.mpc_.r_delta;
    H(i * NU + 1, i * NU + 1) += config_.mpc_.r_a;
  }

  // --- 差分矩阵 T 和控制率惩罚 ---
  // T·ΔU + c 表示绝对控制变化率, 其中
  //   c = [u_ref(0)−u_prev; u_ref(1)−u_ref(0); …]
  Mat T_mat = Mat::eye(nu_total);
  for (int k = 1; k < Nc; ++k) {
    T_mat(k * NU, (k - 1) * NU) = -1.0;
    T_mat(k * NU + 1, (k - 1) * NU + 1) = -1.0;
  }

  Mat c_rate(nu_total, 1);
  c_rate(0, 0) = delta_r[0] - prev_delta_;
  c_rate(1, 0) = accel_r[0] - prev_accel_;
  for (int k = 1; k < Nc; ++k) {
    c_rate(k * NU, 0) = delta_r[k] - delta_r[k - 1];
    c_rate(k * NU + 1, 0) = accel_r[k] - accel_r[k - 1];
  }

  // Tᵀ R̄_d T  和  Tᵀ R̄_d c  (利用 R̄_d 对角结构)
  std::vector<double> rd_w(nu_total);
  for (int k = 0; k < Nc; ++k) {
    rd_w[k * NU] = config_.mpc_.rd_delta;
    rd_w[k * NU + 1] = config_.mpc_.rd_a;
  }

  for (int i = 0; i < nu_total; ++i) {
    for (int j = i; j < nu_total; ++j) {
      double val = 0;
      for (int k = 0; k < nu_total; ++k)
        val += rd_w[k] * T_mat(k, i) * T_mat(k, j);
      H(i, j) += val;
      if (i != j) H(j, i) += val;
    }
    double gval = 0;
    for (int k = 0; k < nu_total; ++k)
      gval += rd_w[k] * T_mat(k, i) * c_rate(k, 0);
    g_vec(i, 0) += gval;
  }

  // 乘以 2 转为标准 QP: min 0.5 ΔUᵀ H ΔU + gᵀ ΔU
  H = H * 2.0;
  g_vec = g_vec * 2.0;

  // ===== 6. 约束 (对控制偏差 ΔU 的 box 约束) =====
  double max_steer = config_.mpc_.max_steering;
  Mat lb(nu_total, 1), ub(nu_total, 1);
  for (int k = 0; k < Nc; ++k) {
    lb(k * NU, 0) = -max_steer - delta_r[std::min(k, Np - 1)];
    ub(k * NU, 0) = max_steer - delta_r[std::min(k, Np - 1)];
    lb(k * NU + 1, 0) = min_accel_ - accel_r[std::min(k, Np - 1)];
    ub(k * NU + 1, 0) = max_accel_ - accel_r[std::min(k, Np - 1)];
  }

  // ===== 7. 热启动 =====
  Mat warm(nu_total, 1);
  if (has_prev_) {
    for (int k = 0; k < Nc - 1; ++k) {
      warm(k * NU, 0) = prev_U_abs_[(k + 1) * NU] - delta_r[k];
      warm(k * NU + 1, 0) = prev_U_abs_[(k + 1) * NU + 1] - accel_r[k];
    }
    warm((Nc - 1) * NU, 0) =
        prev_U_abs_[(Nc - 1) * NU] - delta_r[Nc - 1];
    warm((Nc - 1) * NU + 1, 0) =
        prev_U_abs_[(Nc - 1) * NU + 1] - accel_r[Nc - 1];
  }

  // ===== 8. 求解 QP =====
  Mat dU_opt = solve_box_qp(H, g_vec, lb, ub, warm);

  // ===== 9. 提取首步控制并存储 =====
  double delta_opt = delta_r[0] + dU_opt(0, 0);
  double accel_opt = accel_r[0] + dU_opt(1, 0);

  for (int k = 0; k < Nc; ++k) {
    prev_U_abs_[k * NU] = delta_r[k] + dU_opt(k * NU, 0);
    prev_U_abs_[k * NU + 1] = accel_r[k] + dU_opt(k * NU + 1, 0);
  }
  prev_delta_ = delta_opt;
  prev_accel_ = accel_opt;
  has_prev_ = true;

  // ===== 10. 转向角 → 角速度 =====
  double v_ctrl = std::max(std::abs(v), 0.5);
  angular_velocity = v_ctrl * std::tan(delta_opt) / L;
  acceleration = accel_opt;

  angular_velocity = std::clamp(angular_velocity, min_omega_, max_omega_);
  acceleration = std::clamp(acceleration, min_accel_, max_accel_);
}

}  // namespace Control
