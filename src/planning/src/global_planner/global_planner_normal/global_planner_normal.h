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

  private:
  };

} // namespace Planning
#endif // GLOBAL_PLANNER_NORMAL_H_