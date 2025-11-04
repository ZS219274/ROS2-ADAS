#include "planning_process.h"

#include "config_reader.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process")  //规划总流程
  {
    RCLCPP_INFO(this->get_logger(), "Planning process created");
    auto reader = std::make_shared<ConfigReader>();
    reader->read_planning_process_config();
    auto obs_dis = reader->process().obs_dis_;
    RCLCPP_INFO(this->get_logger(), "obs_dis: %.2f", obs_dis);
  }

  bool PlanningProcess::process()
  {
    return true;
  }
}