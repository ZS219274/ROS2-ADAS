#include "decision_center.h"

namespace Planning
{
  DecisionCenter::DecisionCenter()
  {
    RCLCPP_INFO(rclcpp::get_logger("decision_center.cpp"), "DecisionCenter created");
    decision_config_ = std::make_unique<ConfigReader>();
    decision_config_->read_decision_config();
  }
  void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleInfoBase> &car, const std::vector<std::shared_ptr<VehicleInfoBase>> &obses)
  {
    RCLCPP_INFO(rclcpp::get_logger("decision_center.cpp"), "DecisionCenter make_path_decision");
    // 没有障碍物
    if (obses.empty())
    {
      return;
    }
    // 变道点位初始化
    sl_points_.clear();

    const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;              // 道路左边界
    const double right_bound_l = -decision_config_->pnc_map().road_half_width_ / 2.0;            // 道路右边界
    const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50); // 开始考虑障碍物的范围
    const double least_length = std::max(car->ds_dt() * dis_time, 30.0);                         // 最小变道距离(和30比）
    const double referline_end_length = decision_config_->refer_line().front_size_ *
                                        decision_config_->pnc_map().segment_len_; // 参考线车前长度距离

    // 针对每个障碍物计算变道点位
    SLPoint sl_point; // 变道点位

    for (const auto &obs : obses)
    {
      const double obs_dis_s = obs->s() - car->s();                      // 与障碍物的距离
      if (obs_dis_s > referline_end_length || obs_dis_s < -least_length) // 在车前后范围内考虑障碍物(即便接近终点，也要考虑)
      {
        continue; // 超出考虑范围，跳过
      }

      if (obs->l() > right_bound_l && obs->l() < left_bound_l &&               // 障碍物在车道横向中间
          fabs(obs->dl_dt()) < min_speed && obs->ds_dt() < car->ds_dt() / 2.0) // 侧向速度为0, 纵向速度很慢
      {
        sl_point.s_ = obs->s() + obs->ds_dt() * obs_dis_s / (car->ds_dt() - obs->ds_dt()); // 预测虚拟障碍物点位
        const double obs_left_bound_l = obs->l() + obs->width() / 2.0;                     // 障碍物左边界
        const double obs_right_bound_l = obs->l() - obs->width() / 2.0;                    // 障碍物右边界
        const double left_width = left_bound_l - obs_left_bound_l;                         // 左侧可用宽度
        const double right_width = obs_right_bound_l - right_bound_l;                      // 右侧可用宽度

        if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 如果左边宽度够通过
        {
          sl_point.l_ = (obs_left_bound_l + left_bound_l) / 2.0;
          sl_point.type_ = static_cast<int>(SLPointType::LEFT_PASS);
          sl_points_.emplace_back(sl_point);
        }
        else // 如果左边宽度不够
        {    // 如果右边宽度够通过
          if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)
          {
            sl_point.l_ = (obs_right_bound_l + right_bound_l) / 2.0;
            sl_point.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
            sl_points_.emplace_back(sl_point);
          }
          else // 如果右边宽度不够
          {    // 左右都不够通过，停车点位
            sl_point.l_ = 0.0;
            sl_point.s_ = obs->s() - decision_config_->decision().safe_dis_s_;
            sl_point.type_ = static_cast<int>(SLPointType::STOP);
            sl_points_.emplace_back(sl_point);
            RCLCPP_INFO(rclcpp::get_logger("decision_center.cpp"), "--------- stop, slpoint(s: %.2f, l: %.2f)", sl_point.s_, sl_point.l_);
            break; // 更前方的障碍物这次循环不考虑了
          }
        }
      }
    }
    if (sl_points_.empty())
    {
      return;
    }

    // 头尾的处理
    SLPoint p_start; // 变道过程的起点
    p_start.s_ = sl_points_[0].s_ - least_length;
    p_start.l_ = 0.0;
    p_start.type_ = static_cast<int>(SLPointType::START);
    sl_points_.emplace(sl_points_.begin(), p_start); // 插入起点(头插)

    if (sl_points_.back().type_ != static_cast<int>(SLPointType::STOP))
    {
      SLPoint p_end; // 变道过程的终点
      p_end.s_ = sl_points_.back().s_ + least_length;
      p_end.l_ = 0.0;
      p_end.type_ = static_cast<int>(SLPointType::END);
      sl_points_.emplace_back(p_end); // 插入终点
    }
  }
} // namespace Planning