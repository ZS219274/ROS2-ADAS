#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"

namespace Planning
{
  using base_msgs::msg::PNCMap;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;

  enum class GlobalPlannerType // 全局规划类型
  {
    KNORMAL = 0, // 普通全局路径规划
    KASTAR = 1   // A*全局路径规划
  };

  class GlobalPlannerBase // 全局路径规划基类
  {
  public:
    virtual ~GlobalPlannerBase() {}
    virtual Path search_global_path(const PNCMap &pnc_map) = 0; // 全局路径搜索接口
    inline Path global_path() const { return global_path_; }    // 获取全局路径规划结果路径
  protected:
    std::unique_ptr<ConfigReader> global_planner_config_; // 全局路径规划配置读取器
    int32_t global_plannaer_type_ = 0;                    // 全局路径规划类型
    Path global_path_;                                    // 全局路径规划结果路径
  };
} // namespace Planning
#endif // GLOBAL_PLANNER_BASE_H_