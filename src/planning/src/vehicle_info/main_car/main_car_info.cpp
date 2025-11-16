#include "main_car_info.h"

namespace Planning
{
  MainCar::MainCar()
  {
    RCLCPP_INFO(rclcpp::get_logger("MainCar"), "MainCar created.");
    // 读取配置
    vehicle_config_ = std::make_unique<ConfigReader>();
    vehicle_config_->read_vehicles_config();

    // 更新基本属性
    child_frame_ = vehicle_config_->main_car().frame_;
    length_ = vehicle_config_->main_car().length_;
    width_ = vehicle_config_->main_car().width_;
    speed_ = vehicle_config_->main_car().speed_ori_;
    theta_ = vehicle_config_->main_car().pose_theta_;
    id_ = vehicle_config_->main_car().id_;

    // 初始化定位点
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, theta_);  //绕x y z轴的转角
    local_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
    local_point_.header.stamp = rclcpp::Clock().now();
    local_point_.pose.position.x = vehicle_config_->main_car().pose_x_;
    local_point_.pose.position.y = vehicle_config_->main_car().pose_y_;
    local_point_.pose.position.z = 0.0;
    local_point_.pose.orientation.x = qtn.getX();
    local_point_.pose.orientation.y = qtn.getY();
    local_point_.pose.orientation.z = qtn.getZ();
    local_point_.pose.orientation.w = qtn.getW();
  }
} // namespace Planning