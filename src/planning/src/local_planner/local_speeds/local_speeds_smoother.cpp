#include "local_speeds_smoother.h"

namespace Planning
{
  LocalSpeedsSmoother::LocalSpeedsSmoother() // 局部速度平滑器
  {
    RCLCPP_INFO(rclcpp::get_logger("LocalSpeedsSmoother"), "LocalSpeedsSmoother created.");
  }
} // namespace Planning