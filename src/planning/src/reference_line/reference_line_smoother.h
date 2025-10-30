#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include <Eigen/Dense>                 // Eigen库
#include <OsqpEigen/OsqpEigen.h>        // OSQP求解器
#include <cmath>

namespace Planning {
class ReferenceLineSmoother  //参考线平滑器
{
  public:
    ReferenceLineSmoother();
};

} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_