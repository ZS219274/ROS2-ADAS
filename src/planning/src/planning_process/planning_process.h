#ifndef PLANNING_NODE_H
#define PLANNING_NODE_H

#include "rclcpp/rclcpp.hpp"
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
  class PlanningProcess : public rclcpp::Node //规划总流程
  {
  public:
    PlanningProcess();
    bool process(); // 总流程
  private:
  };
} // namespace Planning
#endif // PLANNING_NODE_H