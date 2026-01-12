#ifndef LOCAL_SPEEDS_PLANNER_H_
#define LOCAL_SPEEDS_PLANNER_H_

#include "config_reader.h"
#include "decision_center.h"
#include "local_speeds_smoother.h"
#include "polynomial_curve.h"
#include "rclcpp/rclcpp.hpp"

namespace Planning {
class LocalSpeedsPlanner  // 局部速度规划器
{
 public:
  LocalSpeedsPlanner();
};
}  // namespace Planning
#endif  // LOCAL_SPEEDS_PLANNER_H_