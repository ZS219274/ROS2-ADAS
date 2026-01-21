#include "config_reader.h"

namespace Control {
ConfigReader::ConfigReader()  // 配置文件读取器
{
  // 获取workspace/install/control/share/control/目录路径
  std::string control_share_directory =
      ament_index_cpp::get_package_share_directory("control");

  // 直接加载control_config.yaml配置文件
  try {
    control_config = YAML::LoadFile(control_share_directory +
                                     "/config/control_config.yaml");
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_config"),
                 "Failed to load control_config.yaml: %s", e.what());
    throw e;
  }
}

/************vehicle*************/
void ConfigReader::read_vehicle_config(VehicleStruct& vehicle,
                                       const std::string& name) {
  vehicle.id_ = control_config["vehicle"][name]["id"].as<int>();
  vehicle.frame_ = control_config["vehicle"][name]["frame"].as<std::string>();
  vehicle.length_ = control_config["vehicle"][name]["length"].as<double>();
  vehicle.width_ = control_config["vehicle"][name]["width"].as<double>();
  vehicle.pose_x_ = control_config["vehicle"][name]["pose_x"].as<double>();
  vehicle.pose_y_ = control_config["vehicle"][name]["pose_y"].as<double>();
  vehicle.pose_theta_ =
      control_config["vehicle"][name]["pose_theta"].as<double>();
  vehicle.speed_ori_ =
      control_config["vehicle"][name]["speed_ori"].as<double>();
}

void ConfigReader::read_vehicles_config() {
  try {
    read_pnc_map_config();
    read_vehicle_config(main_car_, "main_car");
    read_control_config();
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_config"),
                 "Failed to load vehicles config: %s", e.what());
  }
}

/************pnc_map*************/
void ConfigReader::read_pnc_map_config() {
  try {
    pnc_map_.frame_ = control_config["pnc_map"]["frame"].as<std::string>();
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_config"),
                 "Failed to load pnc_map config: %s", e.what());
  }
}

/************control*************/
void ConfigReader::read_control_config() {
  try {
    control_config_.type_ = control_config["control"]["type"].as<int>();
    control_config_.dt_ = control_config["control"]["dt"].as<double>();
    control_config_.lookahead_distance_ =
        control_config["control"]["lookahead_distance"].as<double>();
    control_config_.wheelbase_ =
        control_config["control"]["vehicle"]["wheelbase"].as<double>();

    // 横向控制参数
    control_config_.lateral_.kp_ =
        control_config["control"]["lateral_control"]["kp"].as<double>();
    control_config_.lateral_.ki_ =
        control_config["control"]["lateral_control"]["ki"].as<double>();
    control_config_.lateral_.kd_ =
        control_config["control"]["lateral_control"]["kd"].as<double>();
    control_config_.lateral_.max_output_ =
        control_config["control"]["lateral_control"]["max_output"].as<double>();
    control_config_.lateral_.min_output_ =
        control_config["control"]["lateral_control"]["min_output"].as<double>();

    // 纵向控制参数
    control_config_.longitudinal_.kp_ =
        control_config["control"]["longitudinal_control"]["kp"].as<double>();
    control_config_.longitudinal_.ki_ =
        control_config["control"]["longitudinal_control"]["ki"].as<double>();
    control_config_.longitudinal_.kd_ =
        control_config["control"]["longitudinal_control"]["kd"].as<double>();
    control_config_.longitudinal_.max_output_ =
        control_config["control"]["longitudinal_control"]["max_output"]
            .as<double>();
    control_config_.longitudinal_.min_output_ =
        control_config["control"]["longitudinal_control"]["min_output"]
            .as<double>();
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_config"),
                 "Failed to load control config: %s", e.what());
  }
}
}  // namespace Control

