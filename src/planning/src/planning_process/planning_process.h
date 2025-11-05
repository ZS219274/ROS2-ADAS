#ifndef PLANNING_NODE_H
#define PLANNING_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"

#include "config_reader.h"
#include "main_car/main_car_info.h"
#include "obs_car/obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning
{
  using namespace std::chrono_literals;
  using base_msgs::msg::PNCMap;
  using base_msgs::srv::GlobalPathService;
  using base_msgs::srv::PNCMapService;
  using nav_msgs::msg::Path;

  class PlanningProcess : public rclcpp::Node // 规划总流程
  {
  public:
    PlanningProcess();
    bool process(); // 总流程

  public:
    inline PNCMap pnc_map() const { return pnc_map_; }       // 获取地图
    inline Path global_path() const { return global_path_; } // 获取全局路径

  private:
    bool planning_init();                                     // 规划初始化

    template <typename T>
    bool connect_to_server(const T& client); // 连接服务
    bool pnc_map_request();                    // pnc地图请求
    bool global_path_request();                // 全局路径请求

  private:
    std::unique_ptr<ConfigReader> planning_process_config_;           // 规划总流程配置读取器
    PNCMap pnc_map_;                                                  // 地图
    Path global_path_;                                                // 全局路径
    rclcpp::Client<PNCMapService>::SharedPtr pnc_map_client_;         // pnc地图请求客户端
    rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局路径请求客户端
    double obs_dis_ = 0.0;                                                  // 障碍物距离
  };
} // namespace Planning
#endif // PLANNING_NODE_H