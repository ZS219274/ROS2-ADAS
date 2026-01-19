#include "global_planner_astar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Planning {
AStar::AStar() // A* 全局规划器
{
  RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"),
              "A* global planner created.");
  global_planner_config_ = std::make_unique<ConfigReader>();
  global_planner_config_->read_global_path_config();
  global_plannaer_type_ = static_cast<int32_t>(GlobalPlannerType::KASTAR);
}

void AStar::CrreateGridMap(const PNCMap &pnc_map, double &map_width,
                           double &map_height, double &centor_x,
                           double &centor_y) {
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto &point : pnc_map.midline.points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  // 使用x最大值和最小值差的绝对值作为宽度，y最大值和最小值差的绝对值作为高度
  map_width = std::abs(max_x - min_x);  // x方向范围
  map_height = std::abs(max_y - min_y); // y方向范围
  centor_x = (max_x + min_x) / 2.0;
  centor_y = (max_y + min_y) / 2.0;
}

AStar::StartGoal AStar::calculateStartAndGoal(const PNCMap &pnc_map) {
  StartGoal start_goal;
  // 计算起点坐标（道路中线和右边界的中点），不合理，应该是车辆的坐标
  start_goal.start_x = (pnc_map.midline.points.front().x +
                        pnc_map.right_boundary.points.front().x) /
                       2.0;
  start_goal.start_y = (pnc_map.midline.points.front().y +
                        pnc_map.right_boundary.points.front().y) /
                       2.0;
  // 目标点如果是道路终点其实是不合理的，目标点应该是一个指定的坐标（后期修改）
  start_goal.goal_x = (pnc_map.midline.points.back().x +
                       pnc_map.right_boundary.points.back().x) /
                      2.0;
  start_goal.goal_y = (pnc_map.midline.points.back().y +
                       pnc_map.right_boundary.points.back().y) /
                      2.0;
  return start_goal;
}

Path AStar::createSimplePathFromMidline(const PNCMap &pnc_map) {
  // 设置全局路径的基本信息
  global_path_.header.frame_id = pnc_map.header.frame_id;
  global_path_.header.stamp = rclcpp::Clock().now();
  global_path_.poses.clear();

  // 初始化位姿临时变量
  PoseStamped pose_tmp;
  pose_tmp.header = global_path_.header;
  pose_tmp.pose.orientation.x = 0.0;
  pose_tmp.pose.orientation.y = 0.0;
  pose_tmp.pose.orientation.z = 0.0;
  pose_tmp.pose.orientation.w = 1.0;

  // 根据中线和右边界计算全局路径点，取中点作为路径点
  const int midline_size = pnc_map.midline.points.size();
  for (int i = 0; i < midline_size; i++) {
    pose_tmp.pose.position.x =
        (pnc_map.midline.points[i].x + pnc_map.right_boundary.points[i].x) /
        2.0;
    pose_tmp.pose.position.y =
        (pnc_map.midline.points[i].y + pnc_map.right_boundary.points[i].y) /
        2.0;
    global_path_.poses.emplace_back(pose_tmp);
  }
  return global_path_;
}

AStar::GridMapParams
AStar::initializeGridMapParams(double map_width, double map_height,
                               double centor_x, double centor_y, double scale) {
  const double RESOLUTION = 0.5; // 每个栅格代表0.5米

  // 添加缓冲区确保完全覆盖
  const double GRID_WIDTH = map_width * scale;   // x方向总宽度
  const double GRID_HEIGHT = map_height * scale; // y方向总高度

  // 计算地图原点（确保起点和终点都在地图内）
  double origin_x = centor_x - GRID_WIDTH / 2.0;
  double origin_y = centor_y - GRID_HEIGHT / 2.0;

  GridMapParams params;
  params.origin_x = origin_x;
  params.origin_y = origin_y;
  params.resolution = RESOLUTION;
  params.grid_width = static_cast<int>(GRID_WIDTH / RESOLUTION);
  params.grid_height = static_cast<int>(GRID_HEIGHT / RESOLUTION);

  return params;
}

std::pair<int, int> AStar::worldToGrid(double world_x, double world_y,
                                       const GridMapParams &grid_params) {
  int grid_x = static_cast<int>((world_x - grid_params.origin_x) /
                                grid_params.resolution);
  int grid_y = static_cast<int>((world_y - grid_params.origin_y) /
                                grid_params.resolution);
  return std::make_pair(grid_x, grid_y);
}

int AStar::calculateHeuristic(int x1, int y1, int x2, int y2) {
  int dx = x2 - x1;
  int dy = y2 - y1;
  return static_cast<int>(sqrt(static_cast<double>(dx * dx + dy * dy)) * 10);
}

AStar::Node *
AStar::performAStarSearch(const std::pair<int, int> &start_grid,
                          const std::pair<int, int> &goal_grid,
                          const GridMapParams &grid_params,
                          const std::unordered_set<int> &allowed_points,
                          std::unordered_map<int, Node *> &all_nodes_map) {
  // 初始化开放列表和关闭列表
  auto cmp = [](const Node *a, const Node *b) { return a->f > b->f; };
  std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> open_list(
      cmp);
  std::unordered_set<int> closed_list;
  std::unordered_map<int, Node *> open_list_map;

  // 创建起始节点
  int start_h = calculateHeuristic(start_grid.first, start_grid.second,
                                   goal_grid.first, goal_grid.second);
  int start_key = start_grid.first * grid_params.grid_width + start_grid.second;
  Node *start_node = new Node(start_grid.first, start_grid.second, 0, start_h,
                              start_h, nullptr);
  open_list.push(start_node);
  open_list_map[start_key] = start_node;
  all_nodes_map[start_key] = start_node;

  // 定义8个方向的移动（包括对角线）
  int dx_move[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  int dy_move[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  int move_cost[8] = {10, 14, 10, 14,
                      10, 14, 10, 14}; // 直线代价10，对角线代价14

  Node *goal_node = nullptr;

  // A*主循环
  while (!open_list.empty()) {
    // 取出f值最小的节点
    Node *current = open_list.top();
    open_list.pop();

    // 从开放列表映射中移除
    int current_key = current->x * grid_params.grid_width + current->y;
    open_list_map.erase(current_key);

    // 将当前节点加入关闭列表
    closed_list.insert(current_key);
    // 确保节点在all_nodes_map中
    if (all_nodes_map.find(current_key) == all_nodes_map.end()) {
      all_nodes_map[current_key] = current;
    }

    // 检查是否到达目标点
    if (current->x == goal_grid.first && current->y == goal_grid.second) {
      goal_node = current;
      break;
    }

    // 探索邻居节点
    for (int i = 0; i < 8; i++) {
      int new_x = current->x + dx_move[i];
      int new_y = current->y + dy_move[i];

      // 检查边界
      if (new_x < 0 || new_x >= grid_params.grid_width || new_y < 0 ||
          new_y >= grid_params.grid_height) {
        continue;
      }
      // 检查是否在关闭列表中
      int new_key = new_x * grid_params.grid_width + new_y;
      if (closed_list.find(new_key) != closed_list.end()) {
        continue;
      }
      // 检查是否是可行点（使用预先构建的集合，O(1)查找）
      if (allowed_points.find(new_key) == allowed_points.end()) {
        continue;
      }

      // 计算新节点的g、h、f值
      int new_g = current->g + move_cost[i];
      int new_h =
          calculateHeuristic(new_x, new_y, goal_grid.first, goal_grid.second);
      int new_f = new_g + new_h;

      // 检查该节点是否已在开放列表中
      auto it = open_list_map.find(new_key);
      if (it != open_list_map.end()) {
        // 如果已在开放列表中，检查是否找到了更好的路径
        Node *existing_node = it->second;
        if (new_g < existing_node->g) {
          // 更新现有节点
          existing_node->g = new_g;
          existing_node->f = new_f;
          existing_node->parent = current;
        }
        continue;
      }

      // 创建新节点并加入开放列表
      Node *new_node = new Node(new_x, new_y, new_g, new_h, new_f, current);
      open_list.push(new_node);
      open_list_map[new_key] = new_node;
      all_nodes_map[new_key] = new_node;
    }
  }

  return goal_node;
}

std::vector<std::pair<double, double>>
AStar::buildPathFromNode(Node *goal_node, const GridMapParams &grid_params) {
  std::vector<std::pair<double, double>> waypoints;
  Node *current = goal_node;

  while (current != nullptr) {
    // 将栅格坐标转换为世界坐标
    double world_x = grid_params.origin_x + current->x * grid_params.resolution;
    double world_y = grid_params.origin_y + current->y * grid_params.resolution;
    waypoints.emplace_back(std::make_pair(world_x, world_y));
    current = current->parent;
  }

  // 反转路径（因为是从目标回溯到起点）
  std::reverse(waypoints.begin(), waypoints.end());

  return waypoints;
}

void AStar::cleanupNodes(Node *goal_node,
                         std::unordered_map<int, Node *> &all_nodes_map) {
  // 标记路径上的节点
  std::unordered_set<Node *> path_nodes;
  Node *path_node = goal_node;
  while (path_node != nullptr) {
    path_nodes.insert(path_node);
    path_node = path_node->parent;
  }
  // 删除所有不在路径上的节点
  for (auto &pair : all_nodes_map) {
    if (path_nodes.find(pair.second) == path_nodes.end()) {
      delete pair.second;
    }
  }
}

void AStar::cleanupAllNodes(std::unordered_map<int, Node *> &all_nodes_map) {
  for (auto &pair : all_nodes_map) {
    delete pair.second;
  }
  all_nodes_map.clear();
}

Path AStar::search_global_path(const PNCMap &pnc_map) // 全局路径搜索
{
  RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"),
              "AStar search_global_path!");

  // 计算起点和终点
  StartGoal start_goal = calculateStartAndGoal(pnc_map);

  // 动态重规划机制参数
  const int MAX_RETRY_ATTEMPTS = 3; // 最大重试次数
  const double SCALE_FACTOR = 1.5;  // 每次重试时的地图放大因子
  double map_width = 0.0;
  double map_height = 0.0;
  double centor_x = 0.0;
  double centor_y = 0.0;

  CrreateGridMap(pnc_map, map_width, map_height, centor_x, centor_y);
  if (map_width == 0 || map_height == 0) {
    return createSimplePathFromMidline(pnc_map);
  }

  // 尝试多次路径规划，每次扩大搜索范围
  for (int attempt = 0; attempt <= MAX_RETRY_ATTEMPTS; ++attempt) {
    RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"),
                "AStar path planning attempt: %d", attempt);

    // 根据重试次数调整地图尺寸
    double scale = pow(SCALE_FACTOR, attempt);
    GridMapParams grid_params = initializeGridMapParams(
        map_width, map_height, centor_x, centor_y, scale);

    // 计算起点和终点在栅格地图中的位置
    std::pair<int, int> start_grid =
        worldToGrid(start_goal.start_x, start_goal.start_y, grid_params);
    std::pair<int, int> goal_grid =
        worldToGrid(start_goal.goal_x, start_goal.goal_y, grid_params);

    // 检查起点和终点是否在地图范围内
    if (start_grid.first < 0 || start_grid.first >= grid_params.grid_width ||
        start_grid.second < 0 || start_grid.second >= grid_params.grid_height) {
      RCLCPP_ERROR(rclcpp::get_logger("global_planner_astar.cpp"),
                   "Start point is out of map bounds");
    }

    if (goal_grid.first < 0 || goal_grid.first >= grid_params.grid_width ||
        goal_grid.second < 0 || goal_grid.second >= grid_params.grid_height) {
      RCLCPP_ERROR(rclcpp::get_logger("global_planner_astar.cpp"),
                   "Goal point is out of map bounds");
    }

    // 预先构建允许点的集合（性能优化：避免在A*循环中重复遍历）
    std::unordered_set<int> allowed_points =
        buildAllowedPointsSet(pnc_map, grid_params.origin_x,
                              grid_params.origin_y, grid_params.resolution,
                              grid_params.grid_width, grid_params.grid_height);

    // 执行A*搜索
    std::unordered_map<int, Node *> all_nodes_map;
    Node *goal_node = performAStarSearch(start_grid, goal_grid, grid_params,
                                         allowed_points, all_nodes_map);

    // 构建路径
    global_path_.poses.clear();
    if (goal_node) {
      // 从目标节点回溯构建路径
      std::vector<std::pair<double, double>> waypoints =
          buildPathFromNode(goal_node, grid_params);

      // 填充Path消息
      global_path_.header.frame_id = pnc_map.header.frame_id;
      global_path_.header.stamp = rclcpp::Clock().now();

      PoseStamped pose_tmp;
      pose_tmp.header = global_path_.header;
      pose_tmp.pose.orientation.x = 0.0;
      pose_tmp.pose.orientation.y = 0.0;
      pose_tmp.pose.orientation.z = 0.0;
      pose_tmp.pose.orientation.w = 1.0;

      for (const auto &point : waypoints) {
        pose_tmp.pose.position.x = point.first;
        pose_tmp.pose.position.y = point.second;
        global_path_.poses.emplace_back(pose_tmp);
      }
      RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"),
                  "Path found with %zu waypoints", waypoints.size());

      // 清理内存：保留路径上的节点，删除其他节点
      cleanupNodes(goal_node, all_nodes_map);

      // 成功找到路径，退出循环
      break;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("global_planner_astar.cpp"),
                  "Failed to find a path using A* on attempt %d", attempt);
      // 清理内存（性能优化：防止内存泄漏）
      cleanupAllNodes(all_nodes_map);

      // 如果这是最后一次尝试，记录日志
      if (attempt == MAX_RETRY_ATTEMPTS) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner_astar.cpp"),
                     "Failed to find a path after %d attempts",
                     MAX_RETRY_ATTEMPTS + 1);
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"),
              "AStar global_path created points size: %ld!",
              global_path_.poses.size());
  return global_path_;
}

std::unordered_set<int>
AStar::buildAllowedPointsSet(const PNCMap &pnc_map, double origin_x,
                             double origin_y, double resolution, int grid_width,
                             int grid_height) {
  std::unordered_set<int> allowed_points;
  const auto &midline_points = pnc_map.midline.points;
  const auto &right_boundary_points = pnc_map.right_boundary.points;
  const auto &left_boundary_points = pnc_map.left_boundary.points;

  // 预先计算分辨率平方，用于距离比较（避免sqrt）
  const double resolution_sq = resolution * resolution;

  // 遍历所有道路点，计算允许的栅格坐标
  for (size_t i = 0; i < midline_points.size(); ++i) {
    // 计算三种允许的点：中右中点、中左中点、中线点
    double right_mid_x =
        (midline_points[i].x + right_boundary_points[i].x) / 2.0;
    double right_mid_y =
        (midline_points[i].y + right_boundary_points[i].y) / 2.0;

    double left_mid_x = (midline_points[i].x + left_boundary_points[i].x) / 2.0;
    double left_mid_y = (midline_points[i].y + left_boundary_points[i].y) / 2.0;

    double center_x = midline_points[i].x;
    double center_y = midline_points[i].y;

    // 将世界坐标转换为栅格坐标，并添加到允许集合中
    // 考虑分辨率范围内的所有可能栅格
    int grid_x_right = static_cast<int>((right_mid_x - origin_x) / resolution);
    int grid_y_right = static_cast<int>((right_mid_y - origin_y) / resolution);
    int grid_x_left = static_cast<int>((left_mid_x - origin_x) / resolution);
    int grid_y_left = static_cast<int>((left_mid_y - origin_y) / resolution);
    int grid_x_center = static_cast<int>((center_x - origin_x) / resolution);
    int grid_y_center = static_cast<int>((center_y - origin_y) / resolution);

    // 检查边界并添加到集合（使用平方距离检查，避免sqrt）
    auto addIfValid = [&](int gx, int gy, double wx, double wy) {
      if (gx >= 0 && gx < grid_width && gy >= 0 && gy < grid_height) {
        // 检查栅格中心是否在允许范围内
        double grid_center_x = origin_x + (gx + 0.5) * resolution;
        double grid_center_y = origin_y + (gy + 0.5) * resolution;
        double dx = grid_center_x - wx;
        double dy = grid_center_y - wy;
        if (dx * dx + dy * dy <= resolution_sq) {
          allowed_points.insert(gx * grid_width + gy);
        }
      }
    };

    addIfValid(grid_x_right, grid_y_right, right_mid_x, right_mid_y);
    addIfValid(grid_x_left, grid_y_left, left_mid_x, left_mid_y);
    addIfValid(grid_x_center, grid_y_center, center_x, center_y);

    // 也检查相邻栅格（在分辨率范围内）
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0)
          continue;
        addIfValid(grid_x_right + dx, grid_y_right + dy, right_mid_x,
                   right_mid_y);
        addIfValid(grid_x_left + dx, grid_y_left + dy, left_mid_x, left_mid_y);
        addIfValid(grid_x_center + dx, grid_y_center + dy, center_x, center_y);
      }
    }
  }
  return allowed_points;
}
} // namespace Planning