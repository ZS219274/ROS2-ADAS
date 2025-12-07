#include "reference_line_creator.h"

namespace Planning
{
  ReferenceLineCreator::ReferenceLineCreator()
  {
    RCLCPP_INFO(rclcpp::get_logger("reference_line_creator.cpp"), "ReferenceLineCreator created.");
    // 读取配置文件
    reference_line_config_ = std::make_unique<ConfigReader>();
    reference_line_config_->read_reference_line_config();

    // 创建平滑器
    reference_line_smoother_ = std::make_shared<ReferenceLineSmoother>();
  }

  Referline ReferenceLineCreator::create_reference_line(const Path &global_path,
                                                        const PoseStamped &target_point) // 生成参考线
  {
    // 如果全局路劲为空， 返回空参考线
    if (global_path.poses.empty())
    {
      return refer_line_;
    }

    // 寻找匹配点
    match_point_index_ = Curve::find_match_point(global_path, match_point_index_, target_point);
    last_match_point_index_ = match_point_index_; // 保存刚找到的匹配点到上一帧（迭代）
    if (match_point_index_ < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line_creator.cpp"), "match point index error.");
      return refer_line_;
    }

    // 计算最前点和最后点在全局路径下的坐标
    const int global_path_size = global_path.poses.size();
    front_index_ = (global_path_size - 1 >= match_point_index_ + reference_line_config_->refer_line().front_size_)
                       ? (match_point_index_ + reference_line_config_->refer_line().front_size_)
                       : (global_path_size - 1);
    back_index_ = (0 <= match_point_index_ - reference_line_config_->refer_line().back_size_)
                      ? (match_point_index_ - reference_line_config_->refer_line().back_size_)
                      : 0;

    // 填充参考线
    refer_line_.header.frame_id = reference_line_config_->pnc_map().frame_;
    refer_line_.header.stamp = rclcpp::Clock().now();
    refer_line_.refer_line.clear(); // 参考线初始化完成

    ReferlinePoint point_tmp;
    for (int i = back_index_; i <= front_index_; i++)
    {
      point_tmp.pose = global_path.poses[i];
      refer_line_.refer_line.emplace_back(point_tmp);
    }

    // 平滑参考线
    reference_line_smoother_->smooth_reference_line(refer_line_);

    // 计算投影点参数
    Curve::cal_projection_param(refer_line_);
    RCLCPP_INFO(rclcpp::get_logger("reference_line_creator.cpp"),
                "reference line created, match point index = %d, front index = %d, back index = %d. size = %ld",
                match_point_index_, front_index_, back_index_, refer_line_.refer_line.size());
    return refer_line_;
  }
  Path ReferenceLineCreator::reference_line_to_rviz() // 参考线rviz显示
  {
    refer_line_rviz_.header = refer_line_.header;
    refer_line_rviz_.poses.clear();

    PoseStamped point_tmp;
    for (const auto &point : refer_line_.refer_line)
    {
      point_tmp.header = refer_line_rviz_.header;
      point_tmp.pose = point.pose.pose;
      refer_line_rviz_.poses.emplace_back(point_tmp);
    }
    return refer_line_rviz_;
  }
}