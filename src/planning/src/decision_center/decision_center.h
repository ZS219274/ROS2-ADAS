#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "obs_car/obs_car_info.h"
#include "main_car/main_car_info.h"

namespace Planning
{
  constexpr double min_speed = 0.03;
  enum class SLPointType // 变道点位类型(状态机)
  {
    LEFT_PASS,  // 从左绕
    RIGHT_PASS, // 从右绕
    STOP,       // 停车
    START,      // 整个过程起点
    END         // 终点
  };

  struct SLPoint // 变道点位
  {
    int type_ = 0;
    double s_ = 0.0;
    double l_ = 0.0;
  };

  class DecisionCenter // 决策中心
  {
  public:
    DecisionCenter();

    void make_path_decision(const std::shared_ptr<VehicleInfoBase> &car,
                            const std::vector<std::shared_ptr<VehicleInfoBase>> &obses); // 进行路径决策
    inline std::vector<SLPoint> sl_points() const { return sl_points_; }                 // 获取变道点位
  private:
    std::unique_ptr<ConfigReader> decision_config_; // 配置文件读取器
    std::vector<SLPoint> sl_points_;                // 变道点位
  };

} // namespace Planning
#endif // DECISION_CENTER_H_