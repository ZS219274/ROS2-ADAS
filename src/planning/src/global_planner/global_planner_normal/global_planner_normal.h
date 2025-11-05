#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "rclcpp/rclcpp.hpp"
#include "global_planner_base.h"

namespace Planning
{
  class GlobalPlannerNormal : public GlobalPlannerBase  // 普通全局规划器
  { 
  public:
    GlobalPlannerNormal();
    Path search_global_path(const PNCMap &pnc_map) override;
  private:
  };

} // namespace Planning
#endif // GLOBAL_PLANNER_NORMAL_H_