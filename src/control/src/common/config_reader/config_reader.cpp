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

    // MPC参数（带默认值的安全读取）
    if (control_config["control"]["mpc"]) {
      auto mpc = control_config["control"]["mpc"];
      if (mpc["prediction_horizon"])
        control_config_.mpc_.prediction_horizon =
            mpc["prediction_horizon"].as<int>();
      if (mpc["control_horizon"])
        control_config_.mpc_.control_horizon =
            mpc["control_horizon"].as<int>();
      if (mpc["q_x"])
        control_config_.mpc_.q_x = mpc["q_x"].as<double>();
      if (mpc["q_y"])
        control_config_.mpc_.q_y = mpc["q_y"].as<double>();
      if (mpc["q_theta"])
        control_config_.mpc_.q_theta = mpc["q_theta"].as<double>();
      if (mpc["q_v"])
        control_config_.mpc_.q_v = mpc["q_v"].as<double>();
      if (mpc["r_delta"])
        control_config_.mpc_.r_delta = mpc["r_delta"].as<double>();
      if (mpc["r_a"])
        control_config_.mpc_.r_a = mpc["r_a"].as<double>();
      if (mpc["rd_delta"])
        control_config_.mpc_.rd_delta = mpc["rd_delta"].as<double>();
      if (mpc["rd_a"])
        control_config_.mpc_.rd_a = mpc["rd_a"].as<double>();
      if (mpc["solver_max_iter"])
        control_config_.mpc_.solver_max_iter =
            mpc["solver_max_iter"].as<int>();
      if (mpc["solver_tolerance"])
        control_config_.mpc_.solver_tolerance =
            mpc["solver_tolerance"].as<double>();
      if (mpc["max_steering"])
        control_config_.mpc_.max_steering =
            mpc["max_steering"].as<double>();
    }

    // LQR参数（带默认值的安全读取）
    if (control_config["control"]["lqr"]) {
      auto lqr = control_config["control"]["lqr"];
      if (lqr["lateral"]) {
        auto lat = lqr["lateral"];
        if (lat["q_lateral_error"])
          control_config_.lqr_.q_lateral_error =
              lat["q_lateral_error"].as<double>();
        if (lat["q_heading_error"])
          control_config_.lqr_.q_heading_error =
              lat["q_heading_error"].as<double>();
        if (lat["r_steering"])
          control_config_.lqr_.r_steering = lat["r_steering"].as<double>();
      }
      if (lqr["longitudinal"]) {
        auto lon = lqr["longitudinal"];
        if (lon["q_station_error"])
          control_config_.lqr_.q_station_error =
              lon["q_station_error"].as<double>();
        if (lon["q_velocity_error"])
          control_config_.lqr_.q_velocity_error =
              lon["q_velocity_error"].as<double>();
        if (lon["r_acceleration"])
          control_config_.lqr_.r_acceleration =
              lon["r_acceleration"].as<double>();
      }
      if (lqr["max_iteration"])
        control_config_.lqr_.max_iteration = lqr["max_iteration"].as<int>();
      if (lqr["tolerance"])
        control_config_.lqr_.tolerance = lqr["tolerance"].as<double>();
    }
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_config"),
                 "Failed to load control config: %s", e.what());
  }
}
}  // namespace Control

