#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"

namespace Planning
{
  using base_msgs::msg::Referline;
  using base_msgs::msg::ReferlinePoint;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;

  class ReferenceLineCreator // 参考线创建器
  {
  public:
    ReferenceLineCreator();

    Referline create_reference_line(const Path &global_path,
                                    const PoseStamped &target_point); // 生成参考线
    Path reference_line_to_rviz();             // 参考线rviz显示

  public:
    inline Referline reference_line() const { return refer_line_; }      // 获取参考线
    inline Path reference_line_rviz() const { return refer_line_rviz_; } // 获取参考线rviz显示
    inline int match_point_index() const { return match_point_index_; }  // 获取当前匹配点
    inline int front_index() const { return front_index_; }               // 获取参考线最前点
    inline int back_index() const { return back_index_; }                 // 获取参考线最后点

  private:
    std::unique_ptr<ConfigReader> reference_line_config_;            // 参考线配置
    Referline refer_line_;                                           // 参考线
    Path refer_line_rviz_;                                           // 参考线rviz显示
    std::shared_ptr<ReferenceLineSmoother> reference_line_smoother_; // 参考线平滑
    int last_match_point_index_ = -1;                                // 上一个匹配点索引
    int match_point_index_ = -1;                                     // 当前匹配点索引
    int front_index_ = -1;                                            // 最前点
    int back_index_ = -1;                                             // 最后点
  };

} // namespace Planning
#endif // REFERENCE_LINE_CREATOR_H_