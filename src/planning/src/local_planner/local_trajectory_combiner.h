#ifndef LOCAL_TRAJECTORY_COMBINER_H_
#define LOCAL_TRAJECTORY_COMBINER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "base_msgs/msg/local_trajectory_point.hpp"

#include "config_reader.h"

namespace Planning
{
  using base_msgs::msg::LocalPath;
  using base_msgs::msg::LocalSpeeds;
  using base_msgs::msg::LocalTrajectory;
  using base_msgs::msg::LocalTrajectoryPoint;


  class LocalTrajectoryCombiner   //轨迹合成器
  {
  public:
    LocalTrajectoryCombiner();

    LocalTrajectory combine_trajectory(const LocalPath &local_path, const LocalSpeeds &local_speeds);  // 合成轨迹
    inline LocalTrajectory local_trajectory() const { return local_trajectory_; }  // 获取合成后的轨迹
    
  private:
    // std::unique_ptr<ConfigReader> trajectory_config_;
    LocalTrajectory local_trajectory_;  // 轨迹
  };
} // namespace Planning
#endif // LOCAL_TRAJECTORY_COMBINER_H_