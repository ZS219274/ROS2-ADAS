#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_
#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_planner_normal.h"
#include "global_planner_astar.h"
#include "pnc_map_creator_base.h"

namespace Planning
{
  using base_msgs::srv::GlobalPathService;
  using visualization_msgs::msg::Marker;
  using geometry_msgs::msg::Point;
  using base_msgs::srv::PNCMapService;
  using std::placeholders::_1;
  using std::placeholders::_2;

  class GlobalPathServer : public rclcpp::Node
  {
  public:
    GlobalPathServer();

  private:
    // 全局路径回调
    void response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                       std::shared_ptr<GlobalPathService::Response> response);
    // path转Marker
    Marker Path2Marker(const Path &path);

  private:
    std::shared_ptr<Planning::GlobalPlannerBase> global_planner_creator_; // 全局规划创建器
    rclcpp::Publisher<Path>::SharedPtr global_path_pub_;                  // 全局路径发布器
    rclcpp::Publisher<Marker>::SharedPtr global_path_rviz_pub_;           // 全局路径地图Marker发布器
    rclcpp::Service<GlobalPathService>::SharedPtr global_path_server_;    // 全局路径服务器
  };
} // namespace Planning
#endif // GLOBAL_PATH_SERVER_H_