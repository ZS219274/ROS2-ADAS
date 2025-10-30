#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_
#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning
{
  class PncMapCreatorSturn : public PncMapCreatorBase  //S弯道地图
  {
  public:
    PncMapCreatorSturn();
  };
} // namespace Planning
#endif // PNC_MAP_CREATOR_STRURN_H_