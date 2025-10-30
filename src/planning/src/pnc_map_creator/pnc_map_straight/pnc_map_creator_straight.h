#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_
#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning
{
  class PncMapCreatorStraight : public PncMapCreatorBase  //直道地图
  {
  public:
    PncMapCreatorStraight();
  };

} // namespace Planning
#endif // PNC_MAP_CREATOR_STRAIGHT_H_