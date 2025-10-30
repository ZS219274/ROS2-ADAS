#ifndef LOCAL_SPEEDS_PLANNER_H_
#define LOCAL_SPEEDS_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "decision_center.h"
#include "config_reader.h"
#include "polynomial_curve.h"
#include "local_speeds_smoother.h"


namespace Planning
{
  class LocalSpeedsPlanner // 局部速度规划器
  {
  public:
    LocalSpeedsPlanner();
  };
} // namespace planning
#endif // LOCAL_SPEEDS_PLANNER_H_