#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car/main_car_info.h"

namespace Planning
{
  class CarMoveCmd : public rclcpp::Node
  {
  public:
    CarMoveCmd();
  };
} // namespace Planning

#endif // CAR_MOVE_CMD_H_