#include "global_planner_normal.h"

namespace Planning
{
  GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("GlobalPlannerNormal"), "GlobalPlannerNormal init!");
  }
}