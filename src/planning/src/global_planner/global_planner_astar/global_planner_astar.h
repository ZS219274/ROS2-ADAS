#ifndef GLOBAL_PLANNER_ASTAR_H
#define GLOBAL_PLANNER_ASTAR_H

#include "rclcpp/rclcpp.hpp"
#include "global_planner_base.h"
#include <vector>
#include <utility>

namespace Planning
{
  class AStar : public GlobalPlannerBase
  {
  public:
    AStar();
    Path search_global_path(const PNCMap &pnc_map) override;

  private:
    /**
     * @brief 从道路信息中提取所有合法点位
     * @param pnc_map PNC地图
     * @return 合法点位列表
     */
    std::vector<std::pair<double, double>> extract_valid_points(const PNCMap &pnc_map);
    
    /**
     * @brief 在点列表中查找最近的点
     * @param x 目标点x坐标
     * @param y 目标点y坐标
     * @param points 点列表
     * @return 最近点的索引，如果没找到返回-1
     */
    int find_closest_point(double x, double y, const std::vector<std::pair<double, double>>& points);
    
    /**
     * @brief 基于点位构建道路图
     * @param points 点位列表
     * @return 图的邻接表表示
     */
    std::vector<std::vector<std::pair<int, double>>> build_road_graph(
        const std::vector<std::pair<double, double>>& points);
    
    /**
     * @brief 在点位图上执行A*算法
     * @param graph 图的邻接表表示
     * @param points 点位列表
     * @param start_index 起点索引
     * @param goal_index 终点索引
     * @return 路径点索引列表
     */
    std::vector<int> astar_on_points(
        const std::vector<std::vector<std::pair<int, double>>>& graph,
        const std::vector<std::pair<double, double>>& points,
        int start_index, 
        int goal_index);
    
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