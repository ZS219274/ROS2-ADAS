#include "pnc_map_creator_sturn.h"

namespace Planning
{
  PncMapCreatorSturn::PncMapCreatorSturn()
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map_creator_sturn.cpp"), "PncMapCreatorSturn created.");

    // 读取配置文件, 给参数赋值
    pnc_map_config_ = std::make_unique<ConfigReader>();
    pnc_map_config_->read_pnc_map_config();
    map_type_ = static_cast<int32_t>(PncMapType::kSTURN);

    // 地图起点坐标
    p_mid_.x = -3.0;
    p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;

    // 长度步长  角度步长
    len_step_ = pnc_map_config_->pnc_map().segment_len_;
    theta_step_ = 0.01;

    // 地图初始化
    init_pnc_map();
  }

  PNCMap PncMapCreatorSturn::create_pnc_map()
  {
    draw_straight_x(pnc_map_config_->pnc_map().road_length_ / 3.0, 1.0);
    draw_arc(M_PI_2, 1.0);
    draw_arc(M_PI_2, -1.0);
    draw_straight_x(pnc_map_config_->pnc_map().road_length_ / 3.0, 1.0);
    draw_arc(M_PI_2, -1.0);
    draw_straight_y(pnc_map_config_->pnc_map().road_length_ / 3.0, -1.0);
    draw_arc(M_PI_2, -1.0);
    draw_arc(M_PI_2, 1.0);
    draw_arc(M_PI_2, -1.0);

    // 保证pnc_map_midline.points为偶数，否则rviz无法显示
    if (pnc_map_.midline.points.size() % 2 == 1)
    {
      pnc_map_.midline.points.pop_back();
    }

    // 把所有marker放入pnc_map_markerarray_中
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);

    RCLCPP_INFO(rclcpp::get_logger("pnc_map_creator_sturn.cpp"), "pnc_map created,midline points size: %ld ------ ", pnc_map_.midline.points.size());

    return pnc_map_;
  }
  void PncMapCreatorSturn::init_pnc_map() // 初始化pnc地图
  {
    pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
    pnc_map_.header.stamp = rclcpp::Clock().now();

    pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

    // 中心线格式
    pnc_map_.midline.header = pnc_map_.header;
    pnc_map_.midline.type = Marker::LINE_LIST; // 分段线条
    pnc_map_.midline.ns = "midline";
    pnc_map_.midline.id = 0;
    pnc_map_.midline.action = Marker::ADD;
    pnc_map_.midline.scale.x = 0.05; // 线条宽度
    pnc_map_.midline.color.a = 1.0;  // 线条透明度
    pnc_map_.midline.color.r = 0.7;  // 红色
    pnc_map_.midline.color.g = 0.7;  // 绿色
    pnc_map_.midline.color.b = 0.0;  // 蓝色
    pnc_map_.midline.lifetime = rclcpp::Duration::max();
    pnc_map_.midline.frame_locked = true;

    // 左边界格式
    pnc_map_.left_boundary = pnc_map_.midline;
    pnc_map_.left_boundary.type = Marker::LINE_STRIP; // 连续线条
    pnc_map_.left_boundary.id = 1;
    pnc_map_.left_boundary.color.r = 1.0; // 红色
    pnc_map_.left_boundary.color.g = 1.0; // 绿色
    pnc_map_.left_boundary.color.b = 1.0; // 蓝色

    // 右边界格式
    pnc_map_.right_boundary = pnc_map_.left_boundary;
    pnc_map_.right_boundary.id = 2;
  }
  void PncMapCreatorSturn::draw_straight_x(const double &length, const double &plus_flag, const double &ratio) // 画直道x
  {
    double len_tmp = 0.0;
    while (len_tmp < length)
    {
      pl_.x = p_mid_.x;
      pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;

      pr_.x = p_mid_.x;
      pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;

      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      len_tmp += len_step_ * ratio;
      p_mid_.x += len_step_ * plus_flag * ratio;
    }
  }

  void PncMapCreatorSturn::draw_straight_y(const double &length, const double &plus_flag, const double &ratio) // 画直道y
  {
    double len_tmp = 0.0;
    while (len_tmp < length)
    {
      pl_.x = p_mid_.x + pnc_map_config_->pnc_map().road_half_width_;
      pl_.y = p_mid_.y;

      pr_.x = p_mid_.x - pnc_map_config_->pnc_map().road_half_width_;
      pr_.y = p_mid_.y;

      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      len_tmp += len_step_ * ratio;
      p_mid_.y += len_step_ * plus_flag * ratio;
    }
  }

  void PncMapCreatorSturn::draw_arc(const double &angle, const double &plus_flag, const double &ratio) // 画弧线， 逆时针为正方向，顺时针为负方向， angele为总角度
  {
    double theta_tmp = 0.0;
    while (theta_tmp < angle)
    {
      // 计算 左中右三点坐标
      pl_.x = p_mid_.x - std::sin(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      pl_.y = p_mid_.y + std::cos(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      pr_.x = p_mid_.x + std::sin(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      pr_.y = p_mid_.y - std::cos(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      // 将三点存储
      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      // 中心点前进一步
      double step_x = len_step_ * std::cos(theta_current_);
      double step_y = len_step_ * std::sin(theta_current_);

      p_mid_.x += step_x;
      p_mid_.y += step_y;

      theta_tmp += theta_step_ * ratio;
      theta_current_ += theta_step_ * plus_flag * ratio;
    }
  }
}