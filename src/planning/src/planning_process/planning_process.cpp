#include "planning_process.h"

#include "config_reader.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process") // 规划流程
  {
    RCLCPP_INFO(this->get_logger(), "Planning process created");

    // 读取配置文件
    planning_process_config_ = std::make_unique<ConfigReader>();
    planning_process_config_->read_planning_process_config();
    obs_dis_ = planning_process_config_->process().obs_dis_;

    // 初始化地图服务客户端和全局路径服务客户端
    pnc_map_client_ = this->create_client<PNCMapService>("pnc_map_server");

    global_path_client_ = this->create_client<GlobalPathService>("global_path_server");
  }

  bool PlanningProcess::process() // 总流程
  {
    //阻塞1s, 等待rviz2和xacro模型启动
    rclcpp::Rate rate(1.0);

    // 流程初始化
    if (!planning_init())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize planning process!");
      return false;
    }

    // 进入规划主流程
    

    return true;
  }

  bool PlanningProcess::planning_init() // 规划初始化
  {
    // 生成车辆

    // 连接地图服务器
    if (!connect_to_server(pnc_map_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to pnc_map_server!");
      return false;
    }

    // 获取地图
    if (!pnc_map_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to request pnc_map!");
      return false;
    }

    // 连接全局路径服务器
    if (!connect_to_server(global_path_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to global_path_server!");
      return false;
    }

    // 获取全局路径
    if (!global_path_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to request global_path!");
      return false;
    }
    return true;
  }

  template <typename T>
  bool PlanningProcess::connect_to_server(const T &client) // 连接服务
  {
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for pnc_map_service to be available...");
      return false;
    }
    return true;
  }

  bool PlanningProcess::pnc_map_request() // pnc地图请求
  {
    return true;
  }

  bool PlanningProcess::global_path_request() // 全局路径请求
  {
    return true;
  }

} // namespace Planning