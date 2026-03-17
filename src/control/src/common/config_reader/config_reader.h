#ifndef CONTROL_CONFIG_READER_H_
#define CONTROL_CONFIG_READER_H_

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace Control {
struct VehicleStruct  // 车辆
{
  int id_ = 0;               // 序号
  std::string frame_ = "";   // 坐标系
  double length_ = 0.0;      // 长
  double width_ = 0.0;       // 宽
  double pose_x_ = 0.0;      // x
  double pose_y_ = 0.0;      // y
  double pose_theta_ = 0.0;  // 航向角
  double speed_ori_ = 0.0;   // 初速度
};

struct PNCMapStruct  // 地图
{
  std::string frame_ = "";        // 坐标系
};

struct LateralControlStruct  // 横向控制参数
{
  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double max_output_ = 1.0;
  double min_output_ = -1.0;
};

struct LongitudinalControlStruct  // 纵向控制参数
{
  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double max_output_ = 1.0;
  double min_output_ = -1.0;
};

struct MPCConfigStruct  // MPC控制器参数
{
  int prediction_horizon = 10;     // 预测时域
  int control_horizon = 5;         // 控制时域
  double q_x = 1.0;               // x位置误差权重
  double q_y = 1.0;               // y位置误差权重
  double q_theta = 2.0;           // 航向角误差权重
  double q_v = 1.0;               // 速度误差权重
  double r_delta = 1.0;           // 转向角控制量权重
  double r_a = 1.0;               // 加速度控制量权重
  double rd_delta = 10.0;         // 转向角变化率权重
  double rd_a = 10.0;             // 加速度变化率权重
  int solver_max_iter = 100;       // QP求解器最大迭代
  double solver_tolerance = 1.0e-4;// QP求解器收敛容差
  double max_steering = 0.5;      // 最大转向角 (rad)
};

struct LQRConfigStruct  // LQR控制器参数
{
  double q_lateral_error = 1.0;      // 横向位置误差权重
  double q_heading_error = 3.0;      // 航向角误差权重
  double r_steering = 10.0;          // 转向控制量权重
  double q_station_error = 0.5;      // 纵向位置误差权重
  double q_velocity_error = 2.0;     // 速度误差权重
  double r_acceleration = 5.0;       // 加速度控制量权重
  int max_iteration = 150;           // DARE最大迭代次数
  double tolerance = 1.0e-6;         // DARE收敛容差
};

struct ControlConfigStruct  // 控制配置
{
  int type_ = 0;                       // 控制类型：0-PID, 1-MPC, 2-LQR
  double dt_ = 0.1;                    // 控制周期
  double lookahead_distance_ = 5.0;   // 前视距离
  double wheelbase_ = 2.7;            // 轴距
  double default_speed_ = 10.0;      // 轨迹无速度规划时的默认目标速度(m/s)
  LateralControlStruct lateral_;      // 横向控制参数
  LongitudinalControlStruct longitudinal_;  // 纵向控制参数
  LQRConfigStruct lqr_;              // LQR控制器参数
  MPCConfigStruct mpc_;              // MPC控制器参数
};

class ConfigReader  // 配置文件读取器
{
 public:
  ConfigReader();

  // vehicle
  void read_vehicle_config(VehicleStruct& vehicle, const std::string& name);
  void read_vehicles_config();
  inline VehicleStruct main_car() const { return main_car_; }

  // pnc_map
  void read_pnc_map_config();
  inline PNCMapStruct pnc_map() const { return pnc_map_; }

  // control
  void read_control_config();
  inline ControlConfigStruct control() const { return control_config_; }

 private:
  YAML::Node control_config;

  // vehicle
  VehicleStruct main_car_;

  // pnc_map
  PNCMapStruct pnc_map_;

  // control
  ControlConfigStruct control_config_;
};
}  // namespace Control
#endif  // CONTROL_CONFIG_READER_H_

