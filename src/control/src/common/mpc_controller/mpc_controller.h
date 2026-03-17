#ifndef MPC_CONTROLLER_H_
#define MPC_CONTROLLER_H_

#include <vector>
#include "../controller_base/controller_base.h"
#include "../config_reader/config_reader.h"

namespace Control {

/**
 * @brief 基于线性时变模型的模型预测控制器（LTV-MPC）
 *
 * 在每个控制周期：
 *   1. 沿参考轨迹线性化自行车运动学模型 → 时变 A_k, B_k
 *   2. 构建批量预测模型: Ξ = S_x·ξ₀ + S_u·ΔU
 *   3. 构建二次规划（QP）:
 *        min  Ξᵀ Q̄ Ξ  +  ΔUᵀ R̄ ΔU  +  (T·ΔU+c)ᵀ R̄_d (T·ΔU+c)
 *        s.t. lb ≤ ΔU ≤ ub
 *   4. 投影梯度下降求解 box-constrained QP
 *   5. 取首步控制量: u(0) = u_ref(0) + ΔU(0)
 *   6. 转向角 → 角速度: ω = v·tan(δ)/L
 *
 * 状态: x = [x, y, θ, v]  (NX=4)
 * 控制: u = [δ, a]         (NU=2, δ=转向角, a=加速度)
 */
class MPCController : public ControllerBase {
 public:
  explicit MPCController(const ControlConfigStruct& config);

  void compute_control_inputs(
      const LocalTrajectory::SharedPtr trajectory,
      const VehicleState& vehicle_state,
      int closest_idx, int lookahead_idx,
      double& angular_velocity, double& acceleration) override;

  void reset() override;

 private:
  // ---- 轻量密集矩阵（列优先存储） ----
  struct Mat {
    int r, c;
    std::vector<double> d;
    Mat() : r(0), c(0) {}
    Mat(int r_, int c_) : r(r_), c(c_), d(r_ * c_, 0.0) {}
    double& operator()(int i, int j) { return d[j * r + i]; }
    double  operator()(int i, int j) const { return d[j * r + i]; }
    Mat operator+(const Mat& o) const;
    Mat operator-(const Mat& o) const;
    Mat operator*(const Mat& o) const;
    Mat operator*(double s) const;
    Mat T() const;
    void set_block(int r0, int c0, const Mat& blk);
    Mat block(int r0, int c0, int nr, int nc) const;
    static Mat eye(int n);
    double sqnorm() const;
  };

  static constexpr int NX = 4;
  static constexpr int NU = 2;

  // 自行车运动学模型 Euler 离散线性化
  void linearize(double theta, double v, double delta,
                 Mat& Ad, Mat& Bd) const;

  // 构建批量预测矩阵 S_x (NX·Np × NX), S_u (NX·Np × NU·Nc)
  void build_prediction(const std::vector<Mat>& Ad,
                        const std::vector<Mat>& Bd,
                        Mat& Sx, Mat& Su) const;

  // 幂迭代法估计对称正定矩阵最大特征值
  double spectral_radius(const Mat& H) const;

  // 投影梯度下降求解 box-constrained QP
  Mat solve_box_qp(const Mat& H, const Mat& g,
                   const Mat& lb, const Mat& ub,
                   const Mat& init) const;

  static double norm_angle(double a);

  ControlConfigStruct config_;

  std::vector<double> prev_U_abs_;  // 上次最优绝对控制序列（热启动）
  double prev_delta_;               // 上次执行的转向角
  double prev_accel_;               // 上次执行的加速度
  bool has_prev_;

  double max_omega_, min_omega_;
  double max_accel_, min_accel_;
};

}  // namespace Control

#endif  // MPC_CONTROLLER_H_
