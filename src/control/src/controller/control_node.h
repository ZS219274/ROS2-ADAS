#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

#include <cmath>
#include <memory>
#include <limits>

#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
// 包含 planning 模块的头文件（需要 Planning::ConfigReader）
#include "vehicle_info_base.h"  // Planning::VehicleInfoBase
#include "main_car/main_car_info.h"  // Planning::MainCar
// 包含 Control::ConfigReader（必须在 planning 模块之后，使用完整命名空间避免冲突）
// 注意：由于 include_directories 中 planning 的 config_reader 在前，需要明确指定 control 模块的路径
#include "../common/config_reader/config_reader.h"  // Control::ConfigReader
#include "../common/controller_base/controller_base.h"  // Control::ControllerBase
#include "../common/pid_controller/pid_controller.h"  // Control::PIDControllerWrapper
#include "../common/mpc_controller/mpc_controller.h"  // Control::MPCController
#include "../common/lqr_controller/lqr_controller.h"  // Control::LQRController

namespace Control {
using namespace std::chrono_literals;
using std::placeholders::_1;

using base_msgs::msg::LocalTrajectory;
using geometry_msgs::msg::TransformStamped;
using tf2_ros::TransformBroadcaster;
using Planning::MainCar;

class ControlNode : public rclcpp::Node  // 控制节点
{
 public:
  ControlNode();

 private:
  void trajectory_callback(
      const LocalTrajectory::SharedPtr trajectory);  // 轨迹回调函数

  int find_closest_point(
      const LocalTrajectory::SharedPtr trajectory);  // 查找最近轨迹点

  int find_lookahead_point(
      const LocalTrajectory::SharedPtr trajectory,
      int closest_idx);  // 查找前视点

  void update_vehicle_state(double angular_velocity,
                            double acceleration, double dt);  // 更新车辆状态

  void broadcast_transform();  // 广播坐标变换
  
  void diagnostic_timer_callback();  // 诊断定时器回调

 private:
  std::unique_ptr<Control::ConfigReader> control_config_;  // 控制配置
  std::shared_ptr<TransformBroadcaster> broadcaster_;      // 变换广播器
  rclcpp::Subscription<LocalTrajectory>::SharedPtr
      local_trajectory_sub_;  // 局部轨迹订阅
  std::shared_ptr<MainCar> main_car_;  // 主车,仅用于初始化
  VehicleState vehicle_state_;          // 车辆状态

  // 控制器（通过配置选择PID或MPC）
  std::unique_ptr<ControllerBase> controller_;  // 控制器基类指针

  // 时间管理
  rclcpp::Time last_control_time_;  // 上次控制时间
  rclcpp::Time last_trajectory_time_;  // 上次收到轨迹的时间
  bool first_control_;              // 是否是第一次控制
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;  // 诊断定时器
  int trajectory_received_count_;  // 收到的轨迹消息计数
};
}  // namespace Control
#endif  // CONTROL_NODE_H_

