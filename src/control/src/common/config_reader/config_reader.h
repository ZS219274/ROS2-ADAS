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

struct ControlConfigStruct  // 控制配置
{
  int type_ = 0;                       // 控制类型：0-PID, 1-MPC
  double dt_ = 0.1;                    // 控制周期
  double lookahead_distance_ = 5.0;   // 前视距离
  double wheelbase_ = 2.7;            // 轴距
  LateralControlStruct lateral_;      // 横向控制参数
  LongitudinalControlStruct longitudinal_;  // 纵向控制参数
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

