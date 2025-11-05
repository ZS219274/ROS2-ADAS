#include "global_planner_normal.h"

namespace Planning
{
  GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("GlobalPlannerNormal"), "GlobalPlannerNormal init!");
  }

  Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map) // 全局路径搜索
  {
    (void)pnc_map;
    return global_path_;
  }
}