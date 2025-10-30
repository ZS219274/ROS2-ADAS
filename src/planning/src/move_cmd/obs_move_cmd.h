#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "obs_car/obs_car_info.h"

namespace Planning
{

  class ObsMoveCmd : public rclcpp::Node
  {
  public:
    ObsMoveCmd();
  };
} // namespace Planning
#endif // OBS_MOVE_CMD_H_