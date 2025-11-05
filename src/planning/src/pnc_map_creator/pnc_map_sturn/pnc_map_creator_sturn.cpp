#include "pnc_map_creator_sturn.h"

namespace Planning
{
  PncMapCreatorSturn::PncMapCreatorSturn()
  {
    RCLCPP_INFO(rclcpp::get_logger("PncMapCreatorSturn"), "PncMapCreatorSturn created.");
  }

  PNCMap PncMapCreatorSturn::create_pnc_map()
  {
     return pnc_map_;
  }
}