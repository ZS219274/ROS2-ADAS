#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>

namespace Planning
{
  using  nav_msgs::msg::Path;
  using  geometry_msgs::msg::PoseStamped;
  using  base_msgs::msg::Referline;
  using  base_msgs::msg::LocalPath;

  constexpr double kMathEpsilon = 1.0e-6;

  class Curve // 曲线
  {
  public:
    Curve() = default;

    /** 
     * @brief 寻找匹配点下标
     * @param path 路径
     * @param last_match_point_index 上次匹配点下标
     * @param target_point 车辆位置点
     * @return 匹配点下标
    */
    static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point);


    //计算投影点参数
    static void cal_projection_param(Referline &refer_line); //参考线


  private:
  };
} // namespace Planning
#endif // CURVE_H_