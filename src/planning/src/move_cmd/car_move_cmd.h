#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

#include "config_reader.h"
#include "main_car_info.h"

namespace Planning
{
  using namespace std::chrono_literals;
  using base_msgs::msg::LocalTrajectory;
  using geometry_msgs::msg::TransformStamped;
  using std::placeholders::_1;
  using tf2_ros::TransformBroadcaster;

  struct Car_param
  {
    /*加速度**加加速度*后面看能力自己加*/
    double pos_x_ = 0.0; // 车辆位置x
    double pos_y_ = 0.0; // 车辆位置y
    double theta_ = 0.0; // 车辆航向角
    double speed_ = 0.0; // 车辆速度
  };

  class CarMoveCmd : public rclcpp::Node
  {
  public:
    CarMoveCmd();
  
  private:
   // 广播主车位姿信息
    void car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory); 
  
  private:
    std::unique_ptr<ConfigReader> move_cmd_config_;                   // 配置
    std::shared_ptr<TransformBroadcaster> broadcaster_;               // 广播车辆位姿信息
    rclcpp::Subscription<LocalTrajectory>::SharedPtr trajectory_sub_; // 轨迹订阅器
    std::shared_ptr<VehicleInfoBase> car_;                            // 主车对象（仅用于初始化）
    Car_param car_param_;                                             // 主车信息
  };
} // namespace Planning

#endif // CAR_MOVE_CMD_H_