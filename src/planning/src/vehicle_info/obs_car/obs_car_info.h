#ifndef OBS_CAR_INFO_H_
#define OBS_CAR_INFO_H_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_base.h"
// 除了障碍物车还有创建其他类型的障碍物
namespace Planning
{
  class ObsCar : public VehicleInfoBase // 障碍物车
  {
  public:
    ObsCar(const int &id);

    void vehicle_cartesian_to_frenet(const Referline &refer_line) override;
  };

} // namespace Planning
#endif // OBS_CAR_INFO_H_