#ifndef GLOBAL_PLANNER_ASTAR_H
#define GLOBAL_PLANNER_ASTAR_H

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "global_planner_base.h"
#include "rclcpp/rclcpp.hpp"

namespace Planning {
class AStar : public GlobalPlannerBase {
 public:
  AStar();
  Path search_global_path(const PNCMap& pnc_map) override;

 private:
  struct Node {
    int x, y;      // 栅格坐标
    int g, h, f;   // 代价函数
    Node* parent;  // 父节点

    Node(int x, int y, int g, int h, int f, Node* parent)
        : x(x), y(y), g(g), h(h), f(f), parent(parent) {}
  };

  // 栅格地图参数结构体
  struct GridMapParams {
    double origin_x;
    double origin_y;
    double resolution;
    int grid_width;
    int grid_height;
  };

  // 起点和终点坐标
  struct StartGoal {
    double start_x;
    double start_y;
    double goal_x;
    double goal_y;
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
  std::unordered_set<int> buildAllowedPointsSet(
      const PNCMap& pnc_map, double origin_x, double origin_y,
      double resolution, int grid_width, int grid_height);

  /**
   * @brief 创建栅格地图
   * @param pnc_map PNC地图
   * @param map_width 栅格地图的宽度
   * @param map_height 栅格地图的高度
   * @param centor_x 栅格地图的中心x坐标
   * @param centor_y 栅格地图的中心y坐标
   */
  void CrreateGridMap(const PNCMap& pnc_map, double& map_width,
                      double& map_height, double& centor_x, double& centor_y);

  /**
   * @brief 计算起点和终点坐标
   * @param pnc_map PNC地图
   * @return 起点和终点坐标
   */
  StartGoal calculateStartAndGoal(const PNCMap& pnc_map);

  /**
   * @brief 当地图为空时，根据中线和右边界创建简单路径
   * @param pnc_map PNC地图
   * @return 简单路径
   */
  Path createSimplePathFromMidline(const PNCMap& pnc_map);

  /**
   * @brief 初始化栅格地图参数
   * @param map_width 地图宽度
   * @param map_height 地图高度
   * @param centor_x 地图中心x坐标
   * @param centor_y 地图中心y坐标
   * @param scale 缩放因子
   * @return 栅格地图参数
   */
  GridMapParams initializeGridMapParams(double map_width, double map_height,
                                       double centor_x, double centor_y,
                                       double scale);

  /**
   * @brief 将世界坐标转换为栅格坐标
   * @param world_x 世界坐标x
   * @param world_y 世界坐标y
   * @param grid_params 栅格地图参数
   * @return 栅格坐标 (x, y)
   */
  std::pair<int, int> worldToGrid(double world_x, double world_y,
                                  const GridMapParams& grid_params);

  /**
   * @brief 执行A*搜索算法
   * @param start_grid 起点栅格坐标
   * @param goal_grid 终点栅格坐标
   * @param grid_params 栅格地图参数
   * @param allowed_points 允许的栅格点集合
   * @return 目标节点指针，如果未找到路径则返回nullptr
   */
  Node* performAStarSearch(const std::pair<int, int>& start_grid,
                           const std::pair<int, int>& goal_grid,
                           const GridMapParams& grid_params,
                           const std::unordered_set<int>& allowed_points,
                           std::unordered_map<int, Node*>& all_nodes_map);

  /**
   * @brief 从目标节点回溯构建路径
   * @param goal_node 目标节点
   * @param grid_params 栅格地图参数
   * @return 路径点集合
   */
  std::vector<std::pair<double, double>> buildPathFromNode(
      Node* goal_node, const GridMapParams& grid_params);

  /**
   * @brief 清理所有节点内存（保留路径上的节点）
   * @param goal_node 目标节点
   * @param all_nodes_map 所有节点的映射
   */
  void cleanupNodes(Node* goal_node,
                    std::unordered_map<int, Node*>& all_nodes_map);

  /**
   * @brief 清理所有节点内存（失败情况）
   * @param all_nodes_map 所有节点的映射
   */
  void cleanupAllNodes(std::unordered_map<int, Node*>& all_nodes_map);

  /**
   * @brief 计算启发式代价（曼哈顿距离）
   * @param x1 起点x坐标
   * @param y1 起点y坐标
   * @param x2 终点x坐标
   * @param y2 终点y坐标
   * @return 启发式代价
   */
  int calculateHeuristic(int x1, int y1, int x2, int y2);
};
}  // namespace Planning
#endif  // GLOBAL_PLANNER_ASTAR_H