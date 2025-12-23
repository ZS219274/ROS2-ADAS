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

    draw_straight_x(pnc_map_config_->pnc_map().road_length_ / 2.0, 1.0);

    draw_t_junction(s_edge_point, n_edge_point, e_edge_point, 1.0);
    p_mid_ = e_edge_point;

    draw_crossroads(s_edge_point, n_edge_point, e_edge_point);
    p_mid_ = n_edge_point;

    draw_arc_y_mirror(M_PI_2, 1.0);
    draw_straight_x(pnc_map_config_->pnc_map().road_length_ / 2.0, 1.0);
    draw_arc_x(M_PI_4, -1.0);
    draw_arc_x(M_PI_4, 1.0);
   

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
      if (plus_flag == 1.0)
      {
        pl_.x = p_mid_.x - pnc_map_config_->pnc_map().road_half_width_;
        pl_.y = p_mid_.y;
        pr_.x = p_mid_.x + pnc_map_config_->pnc_map().road_half_width_;
        pr_.y = p_mid_.y;
      }
      if (plus_flag == -1.0)
      {
        pl_.x = p_mid_.x + pnc_map_config_->pnc_map().road_half_width_;
        pl_.y = p_mid_.y;
        pr_.x = p_mid_.x - pnc_map_config_->pnc_map().road_half_width_;
        pr_.y = p_mid_.y;
      }

      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      len_tmp += len_step_ * ratio;
      p_mid_.y += len_step_ * plus_flag * ratio;
    }
  }

  void PncMapCreatorCrossroad::draw_arc_x(const double &angle, const double &plus_flag, const double &ratio) // 画弧线， 逆时针为正方向，顺时针为负方向， angele为总角度
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

  void PncMapCreatorCrossroad::draw_arc_y(const double &angle, const double &plus_flag, const double &ratio) // 沿y方向展开的画弧线
  {
    // 保存原始的theta_current_值
    double original_theta_current = theta_current_;

    // 调整初始角度，使弧线沿y方向展开
    // 如果原来是沿x方向，现在需要旋转90度或-90度来沿y方向展开
    theta_current_ += M_PI_2 * (plus_flag > 0 ? 1 : -1);

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

    // 恢复原始的theta_current_值
    theta_current_ = original_theta_current;
  }

  void PncMapCreatorCrossroad::draw_arc_y_mirror(const double &angle, const double &plus_flag, const double &ratio) // 关于y轴对称的画弧线
  {
    // 保存原始的theta_current_值
    double original_theta_current = theta_current_;

    // 调整初始角度，使弧线沿y方向展开并关于y轴对称
    // 如果原来是沿x方向，现在需要旋转90度或-90度来沿y方向展开
    theta_current_ += M_PI_2 * (plus_flag > 0 ? 1 : -1);

    double theta_tmp = 0.0;
    while (theta_tmp < angle)
    {
      // 计算 左中右三点坐标 (注意左右点的定义也相应改变)
      pl_.x = p_mid_.x - std::sin(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      pl_.y = p_mid_.y + std::cos(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      pr_.x = p_mid_.x + std::sin(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      pr_.y = p_mid_.y - std::cos(theta_current_) * pnc_map_config_->pnc_map().road_half_width_;
      // 将三点存储
      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      // 中心点前进一步 (注意这里也要镜像)
      double step_x = len_step_ * std::cos(theta_current_);
      double step_y = len_step_ * std::sin(theta_current_);

      p_mid_.x += step_x;
      p_mid_.y += step_y;

      theta_tmp += theta_step_ * ratio;
      // 注意这里角度增量也需要调整符号以确保正确的旋转方向
      theta_current_ += theta_step_ * (-plus_flag) * ratio;
    }

    // 恢复原始的theta_current_值
    theta_current_ = original_theta_current;
  }

  void PncMapCreatorCrossroad::draw_crossroads(Point &s_edge_point, Point &n_edge_point, Point &e_edge_point) // 绘制T字型交叉路口
  {

    // 可调整的长度参数（单位与地图一致）
    const double main_up_len = 10.0;      // 主路向上长度
    const double main_down_len = 10.0;    // 主路向下长度
    const double branch_right_len = 20.0; // 支路向右长度
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
  void PncMapCreatorCrossroad::draw_t_junction(Point &s_edge_point, Point &n_edge_point, Point &e_edge_point, const double &plus_flag)
  {
    // 可调整的长度参数（单位与地图一致）
    const double main_up_len = 10.0;      // 主路向上长度
    const double main_down_len = 10.0;    // 主路向下长度
    const double branch_right_len = 20.0; // 支路向右长度
    const double ratio = 1.0;             // 步长比例（与 draw_straight_* 的 ratio 一致）

    // 预先计算常用值，避免重复访问
    const double road_half_width = pnc_map_config_->pnc_map().road_half_width_;
    const double road_width = 2.0 * road_half_width;
    const double step_len = len_step_ * ratio;

    draw_straight_x(branch_right_len, 1.0, ratio);
    Point w_p_tmp = p_mid_;

    Point e_p_tmp = p_mid_;
    e_p_tmp.x += road_width;

    Point n_p_tmp = p_mid_;
    n_p_tmp.x += road_half_width;
    n_p_tmp.y += road_half_width;

    Point s_p_tmp = p_mid_;
    s_p_tmp.x += road_half_width;
    s_p_tmp.y -= road_half_width;

    p_mid_ = e_p_tmp;
    draw_straight_x(branch_right_len, 1.0, ratio);
    e_edge_point = p_mid_;

    // 根据plus_flag选择绘制方向
    const bool is_downward = (plus_flag == -1.0);
    if (is_downward)
    {
      p_mid_ = s_p_tmp;
      draw_straight_y(main_down_len, plus_flag, ratio);
      s_edge_point = p_mid_;
    }
    else
    {
      p_mid_ = n_p_tmp;
      draw_straight_y(main_up_len, plus_flag, ratio);
      n_edge_point = p_mid_;
    }

    // 绘制水平连接段（从w_p_tmp到e_p_tmp）
    double len_tmp = 0.0;
    p_mid_ = w_p_tmp;
    while (len_tmp < road_width)
    {
      len_tmp += step_len;
      p_mid_.x += step_len;
      pl_.x = p_mid_.x;
      pl_.y = p_mid_.y + road_half_width;
      pr_.x = p_mid_.x;
      pr_.y = p_mid_.y - road_half_width;
      pnc_map_.right_boundary.points.emplace_back(pr_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.midline.points.emplace_back(p_mid_);
    }

    // 绘制垂直连接段（根据plus_flag选择起点和方向）
    len_tmp = 0.0;
    p_mid_ = is_downward ? n_p_tmp : s_p_tmp;
    const double y_step = is_downward ? -step_len : step_len;
    
    while (len_tmp < road_width)
    {
      len_tmp += step_len;
      p_mid_.y += y_step;
      pl_.x = p_mid_.x - road_half_width;
      pl_.y = p_mid_.y;
      pr_.x = p_mid_.x + road_half_width;
      pr_.y = p_mid_.y;
      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);
    }
  }
}