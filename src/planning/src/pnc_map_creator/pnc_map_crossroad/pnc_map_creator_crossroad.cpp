#include "pnc_map_creator_crossroad.h"

namespace Planning
{
  PncMapCreatorCrossroad::PncMapCreatorCrossroad()
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map_creator_crossroad.cpp"), "PncMapCreatorCrossroad created.");

    // 读取配置文件, 给参数赋值
    pnc_map_config_ = std::make_unique<ConfigReader>();
    pnc_map_config_->read_pnc_map_config();
    map_type_ = static_cast<int32_t>(PncMapType::kCROSSROAD) + 1; // 在原有基础上增加新的类型

    // 地图起点坐标
    p_mid_.x = -3.0;
    p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;

    // 长度步长  角度步长
    len_step_ = pnc_map_config_->pnc_map().segment_len_;
    theta_step_ = 0.01;

    // 地图初始化
    init_pnc_map();
  }

  PNCMap PncMapCreatorCrossroad::create_pnc_map()
  {
    // 绘制带交叉路口的道路
    Point s_edge_point;
    Point n_edge_point;
    Point e_edge_point;
    draw_crossroad_t(s_edge_point, n_edge_point, e_edge_point);

    // 保证pnc_map_midline.points为偶数，否则rviz无法显示
    if (pnc_map_.midline.points.size() % 2 == 1)
    {
      pnc_map_.midline.points.pop_back();
    }

    // 把所有marker放入pnc_map_markerarray_中
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);

    RCLCPP_INFO(rclcpp::get_logger("pnc_map_creator_crossroad.cpp"), "pnc_map created,midline points size: %ld ------ ", pnc_map_.midline.points.size());

    return pnc_map_;
  }

  void PncMapCreatorCrossroad::init_pnc_map() // 初始化pnc地图
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
    pnc_map_.left_boundary.type = Marker::LINE_LIST; // 连续线条
    pnc_map_.left_boundary.id = 1;
    pnc_map_.left_boundary.color.r = 1.0; // 红色
    pnc_map_.left_boundary.color.g = 1.0; // 绿色
    pnc_map_.left_boundary.color.b = 1.0; // 蓝色

    // 右边界格式
    pnc_map_.right_boundary = pnc_map_.left_boundary;
    pnc_map_.right_boundary.id = 2;
  }

  void PncMapCreatorCrossroad::draw_straight_x(const double &length, const double &plus_flag, const double &ratio) // 画直道x
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

  void PncMapCreatorCrossroad::draw_straight_y(const double &length, const double &plus_flag, const double &ratio) // 画直道y
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

  void PncMapCreatorCrossroad::draw_crossroad_t(Point &s_edge_point, Point n_edge_point, Point e_edge_point) // 绘制T字型交叉路口
  {

    // 可调整的长度参数（单位与地图一致）
    const double main_up_len = 60.0;      // 主路向上长度
    const double main_down_len = 60.0;    // 主路向下长度
    const double branch_right_len = 50.0; // 支路向右长度
    const double ratio = 1.0;             // 步长比例（与 draw_straight_* 的 ratio 一致）

    draw_straight_x(branch_right_len, 1.0, ratio);
    Point w_p_tmp = p_mid_; // 记录十字路口x向西侧中线点

    Point e_p_tmp = w_p_tmp; // 记录十字路口x向东侧中线点
    e_p_tmp.x += 2.0 * pnc_map_config_->pnc_map().road_half_width_;

    Point n_p_tmp = p_mid_; // 记录十字路口y向北侧中线点
    n_p_tmp.x += pnc_map_config_->pnc_map().road_half_width_;
    n_p_tmp.y += pnc_map_config_->pnc_map().road_half_width_;

    Point s_p_tmp = p_mid_; // 记录十字路口y向南侧中线点
    s_p_tmp.x += pnc_map_config_->pnc_map().road_half_width_;
    s_p_tmp.y -= pnc_map_config_->pnc_map().road_half_width_;

    // 绘制主路向下部分
    p_mid_ = s_p_tmp;
    draw_straight_y(main_down_len, -1.0, ratio);
    s_edge_point = p_mid_;

    // 绘制主路向上部分
    p_mid_ = n_p_tmp;
    draw_straight_y(main_up_len, 1.0, ratio);
    n_edge_point = p_mid_;

    // 连接十字路口中心区域与各方向道路
    double len_tmp = 0.0;
    p_mid_ = w_p_tmp;
    while (len_tmp < 2.0 * pnc_map_config_->pnc_map().road_half_width_)
    {
      len_tmp += len_step_ * ratio;
      p_mid_.x += len_step_ * ratio;
      pnc_map_.midline.points.emplace_back(p_mid_);
      // 同时添加对应的左右边界点
      pl_.x = p_mid_.x;
      pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;
      pr_.x = p_mid_.x;
      pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);
    }

    len_tmp = 0.0;
    p_mid_ = s_p_tmp;
    while (len_tmp < 2.0 * pnc_map_config_->pnc_map().road_half_width_)
    {
      len_tmp += len_step_ * ratio;
      p_mid_.y += len_step_ * ratio;
      pnc_map_.midline.points.emplace_back(p_mid_);
      // 同时添加对应的左右边界点
      pl_.x = p_mid_.x + pnc_map_config_->pnc_map().road_half_width_;
      pl_.y = p_mid_.y;
      pr_.x = p_mid_.x - pnc_map_config_->pnc_map().road_half_width_;
      pr_.y = p_mid_.y;
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);
    }

    // 绘制支路向东部分
    p_mid_ = e_p_tmp;
    draw_straight_x(branch_right_len, 1.0, ratio);
    e_edge_point = p_mid_;
  }
}