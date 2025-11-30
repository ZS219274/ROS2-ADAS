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
    // 目标点在笛卡尔下的参数
    struct Cartesian {
      double x;
      double y;
      double theta;
      double speed;
      double a;
      double kappa;
    };
    // 目标点在参考线的投影点在笛卡尔下的参数
    struct Referential {
      double rs;
      double rx;
      double ry;
      double rtheta;
      double rkappa;
      double rdkappa;
    };
    // 目标点在自然系下的参数
    struct Frenet {
      double s;
      double ds_dt;
      double dds_dt;
      double l;
      double dl_ds;
      double dl_dt;
      double ddl_ds;
      double ddl_dt;
    };
  
  using  nav_msgs::msg::Path;
  using  geometry_msgs::msg::PoseStamped;
  using  base_msgs::msg::Referline;
  using  base_msgs::msg::LocalPath;
  constexpr double kMathEpsilon = 1.0e-6;
  constexpr double delta_s_min = 1.0;

  class Curve // 曲线
  {
  public:
    Curve() = default;

    /** 
     * @brief 归一化角度到[-pi, pi)
     * @param angle 输入角度
     * @return 归一化后的角度
    */
    static double NormalizedAngle(const double &angle);

    /** 
     * @brief 笛卡尔转自然坐标系
     * @param cartesian 目标点在笛卡尔坐标系参数
     * @param frenet 目标点在自然坐标系参数
     * @return 无
     */
    static void cartesian_to_frenet(const Cartesian &cartesian, const Referential &ref, Frenet &frenet);

    /** 
     * @brief  自然坐标系转笛卡尔
     * @param frenet 目标点在自然坐标系参数
     * @param cartesian 目标点在笛卡尔坐标系参数
     * @return 无
     */
    static void frenet_to_cartesian(const Frenet &frenet, const Referential &ref, Cartesian &cartesian);

    /** 
     * @brief 利用上一帧匹配点下标寻找匹配点下标
     * @param path 路径
     * @param last_match_point_index 上次匹配点下标
     * @param target_point 车辆位置点
     * @return 匹配点下标
    */
    static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point);

    /**
     * @brief 在参考线上查找匹配点
     * @param path 参考线
     * @param target_point 目标点
     * @return 无
     */
    static int find_match_point(const Referline &refer_line, const PoseStamped &target_point);

    /**
     * @brief 在参考线上查找投影点点
     * @param path 参考线
     * @param target_point 投影点
     * @param ref 投影点在参考线参数
     * @return 无
     */
    static void find_projection_point(const Referline &refer_line, const PoseStamped &target_point, Referential &ref);

    /**
     * @brief 计算投影点参数
     * @param refer_line 参考线
     * @return 无
     */
    static void cal_projection_param(Referline &refer_line); //参考线


  private:
  };
} // namespace Planning
#endif // CURVE_H_