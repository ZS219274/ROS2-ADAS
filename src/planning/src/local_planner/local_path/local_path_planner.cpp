#include "local_path_planner.h"

namespace Planning
{
  LocalPathPlanner::LocalPathPlanner()   // 局部路径规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("LocalPathPlanner"), "LocalPathPlanner created!");
  }
}