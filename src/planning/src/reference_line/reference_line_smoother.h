#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include <Eigen/Dense>           // Eigen库
#include <OsqpEigen/OsqpEigen.h> // OSQP求解器
#include <cmath>
#include "config_reader.h"

namespace Planning
{
  using base_msgs::msg::Referline;
  class ReferenceLineSmoother // 参考线平滑器
  {
  public:
    ReferenceLineSmoother();
    void smooth_reference_line(Referline &refer_line); // 平滑参考线

  private:
    std::unique_ptr<ConfigReader> reference_line_config_; // 配置读取器
    const double w1_ = 100.0;                             // 参考线平滑权重
    const double w2_ = 10.0;                              // 参考线平滑权重
    const double w3_ = 1.0;                               // 参考线平滑权重
  };

} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_