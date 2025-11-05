#include "pnc_map_creator_straight.h"

namespace Planning
{
  PncMapCreatorStraight::PncMapCreatorStraight()
  {
    RCLCPP_INFO(rclcpp::get_logger("PncMapCreatorStraight"), "PncMapCreatorStraight creatd.");
  }

  PNCMap PncMapCreatorStraight::create_pnc_map()
  {
    return pnc_map_; 
  }
}