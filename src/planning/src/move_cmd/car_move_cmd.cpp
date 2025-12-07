#include "car_move_cmd.h"

namespace Planning
{
  CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node")
  {
    RCLCPP_INFO(this->get_logger(), "CarMoveCmd created.");
    move_cmd_config_ = std::make_unique<ConfigReader>();
    move_cmd_config_->read_move_cmd_config();

    // 初始化
    car_ = std::make_shared<MainCar>();
    car_param_.pos_x_ = car_->local_point().pose.position.x;
    car_param_.pos_y_ = car_->local_point().pose.position.y;
    car_param_.theta_ = car_->theta();
    car_param_.speed_ = car_->speed();

    // 创建广播方
    broadcaster_ = std::make_shared<TransformBroadcaster>(this);

    // 创建轨迹订阅方，广播运动指令
    trajectory_sub_ = this->create_subscription<LocalTrajectory>(
        "planning/local_trajectory",
        10,
        std::bind(&CarMoveCmd::car_broadcast_tf, this, _1));
  }
  void CarMoveCmd::car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory)
  {
    // 接收轨迹
    const int trajectory_size = trajectory->local_trajectory.size();
    if (trajectory_size < 3)
    {
      RCLCPP_WARN(this->get_logger(), "CarMoveCmd: received trajectory size less than 3.");
      return;
    }

    // 创建消息对象
    TransformStamped transform_data;
    transform_data.header.stamp = trajectory->header.stamp;
    transform_data.header.frame_id = move_cmd_config_->pnc_map().frame_;
    transform_data.child_frame_id = car_->child_frame();

    // 找到投影点，用匹配点近似替代
    double min_dist = std::numeric_limits<double>::max();
    int cloest_index = -1;
    for (int i = 0; i < trajectory_size; i++)
    {
      double dist = std::hypot(trajectory->local_trajectory[i].path_point.pose.pose.position.x - car_param_.pos_x_,
                               trajectory->local_trajectory[i].path_point.pose.pose.position.y - car_param_.pos_y_);
      if (dist < min_dist)
      {
        min_dist = dist;
        cloest_index = i;
      }
    }
    // 赋值
     // 速度更新
    const double speed_x = 1.0 * std::cos(trajectory->local_trajectory[cloest_index].path_point.theta);
    const double speed_y = 1.0 * std::sin(trajectory->local_trajectory[cloest_index].path_point.theta);

     // 位置更新
#ifdef USE_ACTUAL_POS
       // case 1.  直接用实际点位置赋值。弯道上会有误差
    transform_data.transform.translation.x = car_param_.pos_x_;
    transform_data.transform.translation.y = car_param_.pos_y_;
#else
       // case 2.  用匹配点位置赋值，在加减速时会有误差
    transform_data.transform.translation.x = trajectory->local_trajectory[cloest_index].path_point.pose.pose.position.x;
    transform_data.transform.translation.y = trajectory->local_trajectog
    // 位置更新
    car_param_.pos_x_ += speed_x;
    car_param_.pos_y_ += speed_y;
    // 航向角更新
    transform_data.transform.rotation.x = trajectory->local_trajectory[cloest_index].path_point.pose.pose.orientation.x;
    transform_data.transform.rotation.y = trajectory->local_trajectory[cloest_index].path_point.pose.pose.orientation.y;
    transform_data.transform.rotation.z = trajectory->local_trajectory[cloest_index].path_point.pose.pose.orientation.z;
    transform_data.transform.rotation.w = trajectory->local_trajectory[cloest_index].path_point.pose.pose.orientation.w;
    // 广播消息
    RCLCPP_INFO(this->get_logger(), "move_cmd broadcast, pos: (%.2f, %.2f), speed: (%.2f, %.2f), local_trajectory_size: %ld",
                car_param_.pos_x_, car_param_.pos_y_, speed_x, speed_y, trajectory->local_trajectory.size());
    broadcaster_->sendTransform(transform_data);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::CarMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}