#include "global_planner_normal.h"

namespace Planning
{
  GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("GlobalPlannerNormal"), "GlobalPlannerNormal init!");
    global_planner_config_ = std::make_unique<ConfigReader>();
    global_planner_config_->read_global_path_config();
    global_plannaer_type_ = static_cast<int32_t>(GlobalPlannerType::KNORMAL);
  }

  Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map) // 全局路径搜索
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path"), "GlobalPlannerNormal search_global_path!");
    //普通规划算法，如果是A* 需要封装成函数来调用

    // 设置全局路径的基本信息
    global_path_.header.frame_id = pnc_map.header.frame_id;
    global_path_.header.stamp = rclcpp::Clock().now();
    global_path_.poses.clear();

    // 初始化位姿临时变量
    PoseStamped pose_tmp;
    pose_tmp.header = global_path_.header;
    pose_tmp.pose.orientation.x = 0.0;
    pose_tmp.pose.orientation.y = 0.0;
    pose_tmp.pose.orientation.z = 0.0;
    pose_tmp.pose.orientation.w = 1.0;

     // 根据中线和右边界计算全局路径点，取中点作为路径点
    const int midline_size = pnc_map.midline.points.size();
    for (int i = 0; i < midline_size; i++)
    {
      pose_tmp.pose.position.x = (pnc_map.midline.points[i].x + pnc_map.right_boundary.points[i].x) / 2.0;
      pose_tmp.pose.position.y = (pnc_map.midline.points[i].y + pnc_map.right_boundary.points[i].y) / 2.0;
      global_path_.poses.emplace_back(pose_tmp);
    }
    RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_path created points size: %ld!", global_path_.poses.size());
    return global_path_;
  }
}