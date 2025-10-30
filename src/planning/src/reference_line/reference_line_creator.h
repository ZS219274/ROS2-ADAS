#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_ 

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"


namespace Planning { 
class ReferenceLineCreator  //参考线创建器
{
  public:
    ReferenceLineCreator();
};

} // namespace Planning
#endif // REFERENCE_LINE_CREATOR_H_