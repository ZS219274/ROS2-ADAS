#ifndef MAIN_CAR_INFO_H_
#define MAIN_CAR_INFO_H_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_base.h"

namespace Planning
{
  class MainCar : public VehicleInfoBase // 主车信息
  {
  public:
    MainCar();
  };

} // namespace Planning
#endif // MAIN_CAR_INFO_H_