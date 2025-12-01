#include "local_path_planner.h"

namespace Planning
{
  LocalPathPlanner::LocalPathPlanner() // 局部路径规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("LocalPathPlanner"), "LocalPathPlanner created!");
    local_path_config_ = std::make_unique<ConfigReader>();
    local_path_config_->read_local_path_config();
    local_path_smoother_ = std::make_shared<LocalPathSmoother>();
  }

  LocalPath LocalPathPlanner::create_local_path(const Referline &refer_line,
                                                const std::shared_ptr<VehicleInfoBase> &car,
                                                const std::shared_ptr<DecisionCenter> &decision)
  {
    // 初始化局部路径
    init_local_path();

    // 计算路径点的sl值

    // sl坐标下平滑

    // 转笛卡尔

    // 计算投影点参数

    return local_path_;
  }

  Path LocalPathPlanner::path_to_rviz()
  {
    local_path_rviz_.header = local_path_.header;
    local_path_rviz_.poses.clear();

    PoseStamped point_tmp;
    point_tmp.header = local_path_rviz_.header;
    for (const auto &point : local_path_.local_path)
    {
      point_tmp.pose = point.pose.pose;
      local_path_rviz_.poses.emplace_back(point_tmp);
    }
    return local_path_rviz_;
  }

  void LocalPathPlanner::init_local_path()
  {
    local_path_.header.frame_id = local_path_config_->pnc_map().frame_; // 坐标
    local_path_.header.stamp = rclcpp::Clock().now();                   // 每一帧都更新时间
    local_path_.local_path.clear();                                     // 每一帧都清空，从新规划
  }
} // namespace Planning