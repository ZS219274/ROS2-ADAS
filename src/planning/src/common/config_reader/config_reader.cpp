#include "config_reader.h"

namespace Planning
{
  ConfigReader::ConfigReader() // 配置文件读取器
  {
    // 获取workspace/install/planning/share/planning/目录路径
    std::string planning_share_directory = ament_index_cpp::get_package_share_directory("planning");

    // 然后获取配置文件
    planning_config = YAML::LoadFile(planning_share_directory + "/config/planning_static_obs_config.yaml");
  }

  /************vehicle*************/
  void ConfigReader::read_vehicle_config(VehicleStruct &vehicle, const std::string &name)
  {
    vehicle.id_ = YamlHelper::get_value<int>(planning_config["vehicle"][name], "id", 0);
    vehicle.frame_ = YamlHelper::get_value<std::string>(planning_config["vehicle"][name], "frame", "");
    vehicle.length_ = YamlHelper::get_value<double>(planning_config["vehicle"][name], "length", 0.0);
    vehicle.width_ = YamlHelper::get_value<double>(planning_config["vehicle"][name], "width", 0.0);
    vehicle.pose_x_ = YamlHelper::get_value<double>(planning_config["vehicle"][name], "pose_x", 0.0);
    vehicle.pose_y_ = YamlHelper::get_value<double>(planning_config["vehicle"][name], "pose_y", 0.0);
    vehicle.pose_theta_ = YamlHelper::get_value<double>(planning_config["vehicle"][name], "pose_theta", 0.0);
    vehicle.speed_ori_ = YamlHelper::get_value<double>(planning_config["vehicle"][name], "speed_ori", 0.0);

    if (name != "main_car")
    {
      obs_pair_.emplace(vehicle.id_, vehicle);
    }
  }

  void ConfigReader::read_vehicles_config()
  {
    read_pnc_map_config();
    read_vehicle_config(main_car_, "main_car");
    read_vehicle_config(obs_car1_, "obs_car1");
    read_vehicle_config(obs_car2_, "obs_car2");
    read_vehicle_config(obs_car3_, "obs_car3");
  }

  /************pnc_map*************/
  void ConfigReader::read_pnc_map_config()
  {
    pnc_map_.frame_ = YamlHelper::get_value<std::string>(planning_config["pnc_map"], "frame", "");
    pnc_map_.type_ = YamlHelper::get_value<int>(planning_config["pnc_map"], "type", 0);
    pnc_map_.road_length_ = YamlHelper::get_value<double>(planning_config["pnc_map"], "road_length", 0.0);
    pnc_map_.road_half_width_ = YamlHelper::get_value<double>(planning_config["pnc_map"], "road_half_width", 0.0);
    pnc_map_.segment_len_ = YamlHelper::get_value<double>(planning_config["pnc_map"], "segment_len", 0.0);
    pnc_map_.speed_limit_ = YamlHelper::get_value<double>(planning_config["pnc_map"], "speed_limit", 0.0);
  }

  /************global_path*************/
  void ConfigReader::read_global_path_config()
  {
    global_path_.type_ = YamlHelper::get_value<int>(planning_config["global_path"], "type", 0);
  }

  /************reference_line*************/
  void ConfigReader::read_reference_line_config()
  {
    read_pnc_map_config();
    refer_line_.type_ = YamlHelper::get_value<int>(planning_config["reference_line"], "type", 0);
    refer_line_.front_size_ = YamlHelper::get_value<int>(planning_config["reference_line"], "front_size", 0);
    refer_line_.back_size_ = YamlHelper::get_value<int>(planning_config["reference_line"], "back_size", 0);
  }

  /************local_path*************/
  void ConfigReader::read_local_path_config()
  {
    read_pnc_map_config();
    read_reference_line_config();
    local_path_.curve_type_ = YamlHelper::get_value<int>(planning_config["local_path"], "curve_type", 0);
    local_path_.path_size_ = YamlHelper::get_value<int>(planning_config["local_path"], "path_size", 0);
  }

  /************local_speeds*************/
  void ConfigReader::read_local_speeds_config()
  {
    read_pnc_map_config();
    local_speeds_.speed_size_ = YamlHelper::get_value<int>(planning_config["local_speed"], "speed_size", 0);
  }

  /************decision*************/
  void ConfigReader::read_decision_config()
  {
    read_pnc_map_config();
    read_reference_line_config();
    read_local_path_config();
    read_local_speeds_config();
    decision_.safe_dis_l_ = YamlHelper::get_value<double>(planning_config["decision"], "safe_dis_l", 0.0);
    decision_.safe_dis_s_ = YamlHelper::get_value<double>(planning_config["decision"], "safe_dis_s", 0.0);
  }

  /************planning_process*************/
  void ConfigReader::read_planning_process_config()
  {
    read_pnc_map_config();
    read_global_path_config();
    read_vehicles_config();
    process_.obs_dis_ = YamlHelper::get_value<double>(planning_config["planning_process"], "obs_dis", 0.0);
  }

  /************move_cmd*************/
  void ConfigReader::read_move_cmd_config()
  {
    read_pnc_map_config();
  }
} // namespace Planning
