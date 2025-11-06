#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "config_reader.h"

#include <cmath>

namespace Planning
{
  using base_msgs::msg::PNCMap;
  using visualization_msgs::msg::MarkerArray;
  using visualization_msgs::msg::Marker;
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
    inline MarkerArray pnc_map_mark_array() const { return pnc_map_markerarray_; } // 获取rviz地图

  protected:
    std::unique_ptr<ConfigReader> pnc_map_config_; // pnc地图配置读取器
    int32_t map_type_ = 0;                         // 地图类型
    PNCMap pnc_map_;                               // pnc地图
    MarkerArray pnc_map_markerarray_;               // pnc地图rviz可视化MarkerArray

    Point p_mid_, pl_, pr_;      // 当前点，左点，右点
    double theta_current_ = 0.0; // 当前角度
    double len_step_ = 0.0;      // 长度步长
    double theta_step_ = 0.0;    // 角度步长
  };

} // namespace Planning
#endif // PNC_MAP_CREATOR_BASE_H_