#ifndef PLANNING_NODE_H
#define PLANNING_NODE_H
#include "rclcpp/rclcpp.hpp"

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