#include "obs_car_info.h"

namespace Planning
{
  ObsCar::ObsCar(const int &id)
  {
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "ObsCar created.");
    // 读取配置
    vehicle_config_ = std::make_unique<ConfigReader>();
    vehicle_config_->read_vehicles_config();

    // 更新基本属性
    child_frame_ = vehicle_config_->obs_pair().at(id).frame_;
    length_ = vehicle_config_->obs_pair().at(id).length_;
    width_ = vehicle_config_->obs_pair().at(id).width_;
    theta_ = vehicle_config_->obs_pair().at(id).pose_theta_;
    speed_ = vehicle_config_->obs_pair().at(id).speed_ori_;
    id_ = id;

    // 初始化定位点
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, theta_);
    local_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
    local_point_.header.stamp = rclcpp::Clock().now();
    local_point_.pose.position.x = vehicle_config_->obs_pair().at(id).pose_x_;
    local_point_.pose.position.y = vehicle_config_->obs_pair().at(id).pose_y_;
    local_point_.pose.orientation.x = qtn.x();
    local_point_.pose.orientation.y = qtn.y();
    local_point_.pose.orientation.z = qtn.z();
    local_point_.pose.orientation.w = qtn.w();
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "ObsCar initialized.");
  }

  void ObsCar::vehicle_cartesian_to_frenet(const Referline &refer_line)
  {
    Cartesian cartesian;
    Frenet frenet;
    Referential ref;
    // 计算定位点在参考线上的投影点
    Curve::find_projection_point(refer_line, local_point_, ref);
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "ObsCar projection points: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
      ref.rx, ref.ry, ref.rtheta, ref.rs, ref.rdkappa, ref.rkappa);
    
    // 定义定位点在自然坐标系参数
    cartesian.x = local_point_.pose.position.x;
    cartesian.y = local_point_.pose.position.y;
    cartesian.theta = theta_;
    cartesian.speed = speed_;
    cartesian.kappa = kappa_;
    cartesian.a = acceleration_;
    Curve::cartesian_to_frenet(cartesian, ref, frenet);
    s_ = frenet.s;
    l_ = frenet.l;
    ds_dt_ = frenet.ds_dt;
    dl_ds_ = frenet.dl_ds;
    dl_dt_ = frenet.dl_dt;
    dds_dt_ = frenet.dds_dt;
    ddl_ds_ = frenet.ddl_ds;
    ddl_dt_ = frenet.ddl_dt;

    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "ObsCar frenet: s: %.2f, l: %.2f, ds_dt: %.2f, dl_ds: %.2f, dl_dt: %.2f, dds_dt: %.2f, ddl_ds: %.2f, ddl_dt: %.2f",
      s_, l_, ds_dt_, dl_ds_, dl_dt_, dds_dt_, ddl_ds_, ddl_dt_);
  }

} // namespace Planning