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
    pnc_map_client_ = this->create_client<PNCMapService>("pnc_map_service");

    global_path_client_ = this->create_client<GlobalPathService>("global_path_service");
  }

  bool PlanningProcess::process() // 总流程
  {
    // 阻塞1s, 等待rviz2和xacro模型启动
    rclcpp::Rate rate(1.0);
    rate.sleep();

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
    // 服务器名称
    std::string server_name;

    // 判断客户端类型
    if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>) // c++17引入的新特性
    {
      server_name = "pnc_map_server";
    }
    else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
    {
      server_name = "global_path_server";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid client type!");
      return false;
    }

    // 等待服务器连接
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s. Exiting.", server_name.c_str());
        return false;
      }
      RCLCPP_WARN(this->get_logger(), "Waiting for %s to be available...", server_name.c_str());
    }
    return true;
  }

  bool PlanningProcess::pnc_map_request() // pnc地图请求
  {
    RCLCPP_INFO(this->get_logger(), "Requesting pnc_map from pnc_map_server...");
    // 发送请求
    auto request = std::make_shared<PNCMapService::Request>();
    request->map_type = planning_process_config_->pnc_map().type_;

    // 生成响应
    auto result_future = pnc_map_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "pnc_map response success.");
      pnc_map_ = result_future.get()->pnc_map;
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "pnc_map response success failed.");
      return false;
    }
    return false;
  }

  bool PlanningProcess::global_path_request() // 全局路径请求
  {
    RCLCPP_INFO(this->get_logger(), "Requesting global_path from global_path_server...");

    // 发送请求
    auto request = std::make_shared<GlobalPathService::Request>();
    request->pnc_map = pnc_map_;
    request->global_planner_type = planning_process_config_->global_path().type_;

    // 生成响应
    auto result_future = global_path_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "global_path response success.");
      global_path_ = result_future.get()->global_path;
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "global_path response success failed.");
      return false;
    }
    return false;
  }

} // namespace Planning