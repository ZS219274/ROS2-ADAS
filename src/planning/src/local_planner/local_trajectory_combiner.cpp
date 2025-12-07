#include "local_trajectory_combiner.h"

namespace Planning
{
  LocalTrajectoryCombiner::LocalTrajectoryCombiner()   //轨迹合成器
  {
    RCLCPP_INFO(rclcpp::get_logger("local_trajectory_combiner.cpp"), "LocalTrajectoryCombiner init.");
    // trajectory_config_ = std::make_unique<ConfigReader>();
  }

  LocalTrajectory LocalTrajectoryCombiner::combine_trajectory(const LocalPath &local_path, const LocalSpeeds &local_speeds)
  {
    const int path_size = local_path.local_path.size();
    const int speeds_size = local_speeds.local_speeds.size();
    local_trajectory_.header = local_path.header;
    local_trajectory_.local_trajectory.clear();

    if (path_size < 3 || speeds_size < 3)
    {
      RCLCPP_WARN(rclcpp::get_logger("local_trajectory_combiner.cpp"), "LocalTrajectoryCombiner: path or speeds size is less than 3.");
      return local_trajectory_;
    }

    LocalTrajectoryPoint point_tmp;
    for (int i = 0; i < path_size; i++)
    {
      // 路径部分填充
      point_tmp.path_point = local_path.local_path[i];
      
      // 速度部分填充
      // point_tmp.speed_point = local_speeds.local_speeds[i];

      // 轨迹填充
      local_trajectory_.local_trajectory.emplace_back(point_tmp);
    }
    RCLCPP_INFO(rclcpp::get_logger("local_trajectory_combiner.cpp"), "LocalTrajectory Combined success!, size: %ld", local_trajectory_.local_trajectory.size());
    return local_trajectory_;
  }

  

} // namespace Planning