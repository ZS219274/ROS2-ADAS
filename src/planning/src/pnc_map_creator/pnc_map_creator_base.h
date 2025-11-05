#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "config_reader.h"

namespace Planning
{
  using base_msgs::msg::PNCMap;
  using visualization_msgs::msg::MarkerArray;
  using geometry_msgs::msg::Point;

  enum class PncMapType // pnc地图类型
  {
    KSTRAIGHT = 0, // 普通直线道路
    kSTURN = 1     // S形道路
  };

  class PncMapCreatorBase // pnc地图创建器基类
  {
  public:
    virtual PNCMap create_pnc_map() = 0; // 生成地图
    virtual ~PncMapCreatorBase() {}
    inline PNCMap pnc_map() const { return pnc_map_; }                            // 获取地图
    inline MarkerArray pnc_map_mark_array() const { return pnc_map_mark_array_; } // 获取rviz地图

  protected:
    std::unique_ptr<ConfigReader> pnc_map_config_; // pnc地图配置读取器
    int32_t map_type_ = 0;                         // 地图类型
    PNCMap pnc_map_;                               // pnc地图
    MarkerArray pnc_map_mark_array_;               // pnc地图rviz可视化MarkerArray
    Point p_mid_;                                  // 车道中心点
    Point p_r_;                                    // 车道右边界点
    Point p_l_;                                    // 车道左边界点
    double currentt_theta_;                        // 当前车辆航向角
    double len_step_;                              // 轨迹点间隔
    double theta_step_;                            // 航向角变化步长
  };

} // namespace Planning
#endif // PNC_MAP_CREATOR_BASE_H_