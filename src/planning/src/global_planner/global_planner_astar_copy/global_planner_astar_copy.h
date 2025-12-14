#ifndef GLOBAL_PLANNER_ASTAR_H
#define GLOBAL_PLANNER_ASTAR_H

#include "rclcpp/rclcpp.hpp"
#include "global_planner_base.h"

namespace Planning
{
  class AStar : public GlobalPlannerBase
  {
  public:
    AStar();
    Path search_global_path(const PNCMap &pnc_map) override;

  private:
    struct Node
    {
      int x, y;      // 栅格坐标
      int g, h, f;   // 代价函数
      Node* parent;  // 父节点
      
      Node(int x, int y, int g, int h, int f, Node* parent)
          : x(x), y(y), g(g), h(h), f(f), parent(parent) {}
    };
    
    /**
     * @brief 检查点是否是允许的可行走点
     * @param x 点的x坐标
     * @param y 点的y坐标
     * @param pnc_map PNC地图
     * @param resolution 地图分辨率
     * @return 如果点是允许的可行走点则返回true，否则返回false
     */
    bool isPointAllowed(double x, double y, const PNCMap &pnc_map, double resolution);
  };
} // namespace Planning

#endif // GLOBAL_PLANNER_ASTAR_H