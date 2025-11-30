#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "curve.h"

namespace Planning
{
  using base_msgs::msg::Referline;
  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TransformStamped;
  using nav_msgs::msg::Path;

  class VehicleInfoBase // 车辆信息基类
  {
  public:
    //更新参数
    inline void update_location(const PoseStamped &loc) { local_point_ = loc; }

    //定位点转frenet
    virtual void  vehicle_cartesian_to_frenet(const Referline &refer_line) = 0;

    //基本属性
    inline std::string child_frame() const { return child_frame_; }
    inline double length() const { return length_; }
    inline double width() const { return width_; }
    inline int id() const { return id_; }

    // 笛卡尔参数
    inline PoseStamped local_point() const { return local_point_; }
    inline double theta() const { return theta_; }
    inline double kappa() const { return kappa_; }
    inline double dkappa() const { return dkappa_; }
    inline double speed() const { return speed_; }
    inline double acceleration() const { return acceleration_; }
    inline double dacceleration() const { return dacceleration_; }

    // 向参考线投影的frenet参数
    inline double s() const { return s_; }
    inline double l() const { return l_; }
    inline double ds_dt() const { return ds_dt_; }
    inline double dl_ds() const { return dl_ds_; }
    inline double dl_dt() const { return dl_dt_; }
    inline double ddl_ds() const { return ddl_ds_; }
    inline double dds_dt() const { return dds_dt_; }
    inline double ddl_dt() const { return ddl_dt_; }
    
    // 向路径投影的frenet参数

    // 时间参数


    //虚析构
    virtual ~VehicleInfoBase() {}

    

  protected:
    //基本属性
    std::unique_ptr<ConfigReader> vehicle_config_; // 车辆信息配置
    std::string child_frame_;                      // 子坐标系
    double length_ = 0.0;                          // 车辆长度
    double width_ = 0.0;                           // 车辆宽度
    int id_ = 0;                                   // 车辆序号id

    // 笛卡尔参数
    PoseStamped local_point_;    // 车辆位姿
    double theta_ = 0.0;         // 航向角
    double kappa_ = 0.0;         // 曲率
    double dkappa_ = 0.0;        // 曲率变化率
    double speed_ = 0.0;         // 速度
    double acceleration_ = 0.0;  // 加速度
    double dacceleration_ = 0.0; // 加加速度

    // 向参考线投影的frenet参数
    double s_ = 0.0;
    double l_ = 0.0;
    double ds_dt_ = 0.0;
    double dl_ds_ = 0.0;
    double dl_dt_ = 0.0;
    double dds_dt_ = 0.0;
    double ddl_ds_ = 0.0;
    double ddl_dt_ = 0.0;

    // 向路径投影的frenet参数
    

    // 时间参数
  };

} // namespace Planning
#endif // VEHICLE_INFO_BASE_H_