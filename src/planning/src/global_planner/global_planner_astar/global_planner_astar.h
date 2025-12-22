#ifndef GLOBAL_PLANNER_ASTAR_H
#define GLOBAL_PLANNER_ASTAR_H

#include "rclcpp/rclcpp.hpp"
#include "global_planner_base.h"
#include <unordered_set>

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
      int x, y;     // 栅格坐标
      int g, h, f;  // 代价函数
      Node *parent; // 父节点

      Node(int x, int y, int g, int h, int f, Node *parent)
          : x(x),
            y(y),
            g(g),
            h(h),
            f(f),
            parent(parent) {}
    };

    /**
     * @brief 预先构建允许点的栅格坐标集合（性能优化）
     * @param pnc_map PNC地图
     * @param origin_x 地图原点x坐标
     * @param origin_y 地图原点y坐标
     * @param resolution 地图分辨率
     * @param grid_width 栅格地图宽度
     * @param grid_height 栅格地图高度
     * @return 允许点的栅格坐标集合（key = x * grid_width + y）
     */
    std::unordered_set<int> buildAllowedPointsSet(const PNCMap &pnc_map,
                                                  double origin_x, double origin_y,
                                                  double resolution,
                                                  int grid_width, int grid_height);
    /**
     * @brief 创建栅格地图
     * @param pnc_map PNC地图
     * @param map_width 栅格地图的宽度
     * @param map_height 栅格地图的高度
     * @param centor_x 栅格地图的中心x坐标
     * @param centor_y 栅格地图的中心y坐标
     */
    void CrreateGridMap(const PNCMap &pnc_map,
                        double &map_width,
                        double &map_height,
                        double &centor_x,
                        double &centor_y);
  };
} // namespace Planning
#endif // GLOBAL_PLANNER_ASTAR_H