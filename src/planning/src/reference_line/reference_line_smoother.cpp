#include "reference_line_smoother.h"

namespace Planning
{
  ReferenceLineSmoother::ReferenceLineSmoother()  //参考线平滑器
  {
    RCLCPP_INFO(rclcpp::get_logger("ReferenceLineSmoother"), "ReferenceLineSmoother created.");
  }
} // namespace Planning