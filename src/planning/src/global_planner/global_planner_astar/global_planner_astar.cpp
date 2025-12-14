#include "global_planner_astar.h"
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <memory>

namespace Planning
{
  AStar::AStar()  // A* 全局规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "A* global planner created.");
    global_planner_config_ = std::make_unique<ConfigReader>();
    global_planner_config_->read_global_path_config();
    global_plannaer_type_ = static_cast<int32_t>(GlobalPlannerType::KASTAR);
  }

  Path AStar::search_global_path(const PNCMap &pnc_map) // 全局路径搜索
  {
    RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "Point-based AStar search_global_path!");
    
    // 基于车道合法点位的A*算法:
    // 1. 预先提取所有合法点位（车道上的点）
    // 2. 构建这些点之间的连接关系
    // 3. 在这个点集合上运行A*算法
    
    // 提取所有合法点位
    std::vector<std::pair<double, double>> valid_points = extract_valid_points(pnc_map);
    
    if (valid_points.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("AStar"), "No valid points found on the road!");
      return global_path_;
    }
    
    // 确定起点和终点
    double start_x = (pnc_map.midline.points.front().x + pnc_map.right_boundary.points.front().x) / 2.0;
    double start_y = (pnc_map.midline.points.front().y + pnc_map.right_boundary.points.front().y) / 2.0;
    double goal_x = (pnc_map.midline.points.back().x + pnc_map.right_boundary.points.back().x) / 2.0;
    double goal_y = (pnc_map.midline.points.back().y + pnc_map.right_boundary.points.back().y) / 2.0;
    
    // 查找最近的合法点作为实际的起点和终点
    int start_index = find_closest_point(start_x, start_y, valid_points);
    int goal_index = find_closest_point(goal_x, goal_y, valid_points);
    
    if (start_index == -1 || goal_index == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("AStar"), "Failed to find start or goal point among valid points!");
      return global_path_;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("AStar"), "Found %zu valid points, start index: %d, goal index: %d", 
                valid_points.size(), start_index, goal_index);
    
    // 构建邻接表表示的图
    std::vector<std::vector<std::pair<int, double>>> graph = build_road_graph(valid_points);
    
    // 运行A*算法
    std::vector<int> path_indices = astar_on_points(graph, valid_points, start_index, goal_index);
    
    if (path_indices.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("AStar"), "Failed to find path using point-based A*!");
      return global_path_;
    }
    
    // 构建最终路径
    global_path_.header.frame_id = pnc_map.header.frame_id;
    global_path_.header.stamp = rclcpp::Clock().now();
    
    PoseStamped pose_tmp;
    pose_tmp.header = global_path_.header;
    pose_tmp.pose.orientation.x = 0.0;
    pose_tmp.pose.orientation.y = 0.0;
    pose_tmp.pose.orientation.z = 0.0;
    pose_tmp.pose.orientation.w = 1.0;
    
    for (int idx : path_indices) {
      pose_tmp.pose.position.x = valid_points[idx].first;
      pose_tmp.pose.position.y = valid_points[idx].second;
      global_path_.poses.emplace_back(pose_tmp);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("AStar"), "Point-based A* path found with %zu waypoints", path_indices.size());
    return global_path_;
  }
  
  std::vector<std::pair<double, double>> AStar::extract_valid_points(const PNCMap &pnc_map)
  {
    std::vector<std::pair<double, double>> valid_points;
    
    const auto& midline_points = pnc_map.midline.points;
    const auto& right_boundary_points = pnc_map.right_boundary.points;
    
    // 遍历所有道路点，只提取右侧合法点位
    for (size_t i = 0; i < midline_points.size(); ++i) {
      // 只提取中线和右边界的中点（右侧车道中心）
      double right_mid_x = (midline_points[i].x + right_boundary_points[i].x) / 2.0;
      double right_mid_y = (midline_points[i].y + right_boundary_points[i].y) / 2.0;
      valid_points.emplace_back(right_mid_x, right_mid_y);
    }
    
    return valid_points;
  }
  
  int AStar::find_closest_point(double x, double y, const std::vector<std::pair<double, double>>& points)
  {
    if (points.empty()) {
      return -1;
    }
    
    int closest_index = 0;
    double min_distance = sqrt(pow(x - points[0].first, 2) + pow(y - points[0].second, 2));
    
    for (size_t i = 1; i < points.size(); ++i) {
      double distance = sqrt(pow(x - points[i].first, 2) + pow(y - points[i].second, 2));
      if (distance < min_distance) {
        min_distance = distance;
        closest_index = static_cast<int>(i);
      }
    }
    
    return closest_index;
  }
  
  std::vector<std::vector<std::pair<int, double>>> AStar::build_road_graph(
      const std::vector<std::pair<double, double>>& points)
  {
    const double MAX_CONNECTION_DISTANCE = 5.0; // 最大连接距离
    std::vector<std::vector<std::pair<int, double>>> graph(points.size());
    
    // 为每个点建立与其他点的连接
    for (size_t i = 0; i < points.size(); ++i) {
      for (size_t j = 0; j < points.size(); ++j) {
        if (i == j) continue;
        
        double distance = sqrt(pow(points[i].first - points[j].first, 2) + 
                              pow(points[i].second - points[j].second, 2));
        
        // 只连接相对较近的点
        if (distance <= MAX_CONNECTION_DISTANCE) {
          graph[i].emplace_back(j, distance);
        }
      }
    }
    
    return graph;
  }
  
  std::vector<int> AStar::astar_on_points(
      const std::vector<std::vector<std::pair<int, double>>>& graph,
      const std::vector<std::pair<double, double>>& points,
      int start_index, 
      int goal_index)
  {
    struct PointNode {
      int index;
      double g, h, f;
      int parent;
      
      PointNode() : index(-1), g(0.0), h(0.0), f(0.0), parent(-1) {}  // 默认构造函数
      
      PointNode(int idx, double g, double h, double f, int parent)
          : index(idx), g(g), h(h), f(f), parent(parent) {}
    };
    
    auto cmp = [](const PointNode& a, const PointNode& b) { return a.f > b.f; };
    std::priority_queue<PointNode, std::vector<PointNode>, decltype(cmp)> open_list(cmp);
    std::unordered_set<int> closed_list;
    std::unordered_map<int, PointNode> open_list_map;
    
    // 启发式函数：欧几里得距离
    auto heuristic = [&](int from, int to) {
      return sqrt(pow(points[from].first - points[to].first, 2) + 
                 pow(points[from].second - points[to].second, 2));
    };
    
    // 初始化起点
    double start_h = heuristic(start_index, goal_index);
    PointNode start_node(start_index, 0.0, start_h, start_h, -1);
    open_list.push(start_node);
    open_list_map[start_index] = start_node;
    
    PointNode* goal_node = nullptr;
    std::unordered_map<int, PointNode> all_nodes;
    all_nodes[start_index] = start_node;
    
    // A*主循环
    while (!open_list.empty()) {
      PointNode current = open_list.top();
      open_list.pop();
      open_list_map.erase(current.index);
      closed_list.insert(current.index);
      
      // 检查是否到达目标点
      if (current.index == goal_index) {
        goal_node = new PointNode(current.index, current.g, current.h, current.f, current.parent);
        all_nodes[current.index] = current;
        break;
      }
      
      // 探索邻居节点
      for (const auto& neighbor : graph[current.index]) {
        int neighbor_index = neighbor.first;
        double move_cost = neighbor.second;
        
        // 检查是否在关闭列表中
        if (closed_list.find(neighbor_index) != closed_list.end()) {
          continue;
        }
        
        // 计算新节点的g、h、f值
        double new_g = current.g + move_cost;
        double new_h = heuristic(neighbor_index, goal_index);
        double new_f = new_g + new_h;
        
        // 检查该节点是否已在开放列表中
        auto it = open_list_map.find(neighbor_index);
        if (it != open_list_map.end()) {
          // 如果已在开放列表中，检查是否找到了更好的路径
          PointNode existing_node = it->second;
          if (new_g < existing_node.g) {
            // 更新现有节点
            existing_node.g = new_g;
            existing_node.f = new_f;
            existing_node.parent = current.index;
            open_list_map[neighbor_index] = existing_node;
          }
          continue;
        }
        
        // 创建新节点并加入开放列表
        PointNode new_node(neighbor_index, new_g, new_h, new_f, current.index);
        open_list.push(new_node);
        open_list_map[neighbor_index] = new_node;
        all_nodes[neighbor_index] = new_node;
      }
    }
    
    // 构建路径
    std::vector<int> path_indices;
    if (goal_node) {
      // 从目标节点回溯到起始节点构建路径
      int current_index = goal_node->index;
      while (current_index != -1) {
        path_indices.push_back(current_index);
        current_index = all_nodes[current_index].parent;
      }
      
      // 反转路径（因为是从目标回溯到起点）
      std::reverse(path_indices.begin(), path_indices.end());
    }
    
    delete goal_node;
    return path_indices;
  }
  
  bool AStar::isPointAllowed(double x, double y, const PNCMap &pnc_map, double resolution)
  {
    // 仅允许在右侧车道上的点移动：
    // 道路中线和右边界的中点
    
    (void)resolution;
    const auto& midline_points = pnc_map.midline.points;
    const auto& right_boundary_points = pnc_map.right_boundary.points;
    // 不再检查左边界点，注释掉相关代码
    // const auto& left_boundary_points = pnc_map.left_boundary.points;
    
    // 遍历所有道路点，检查给定点是否接近右侧车道中心点
    for (size_t i = 0; i < midline_points.size(); ++i) {
      // 只检查是否接近中线和右边界的中点（右侧车道中心）
      double right_mid_x = (midline_points[i].x + right_boundary_points[i].x) / 2.0;
      double right_mid_y = (midline_points[i].y + right_boundary_points[i].y) / 2.0;
      
      // 不再检查左侧点位，注释掉相关代码
      // double left_mid_x = (midline_points[i].x + left_boundary_points[i].x) / 2.0;
      // double left_mid_y = (midline_points[i].y + left_boundary_points[i].y) / 2.0;
      
      // 检查给定点是否足够接近右侧车道中心点（在道路半宽范围内）
      double dist_to_right = sqrt(pow(x - right_mid_x, 2) + pow(y - right_mid_y, 2));
      // 不再检查到左侧点的距离，注释掉相关代码
      // double dist_to_left = sqrt(pow(x - left_mid_x, 2) + pow(y - left_mid_y, 2));
      
      // 如果点足够接近右侧车道中心点，则认为是可行的
      if (dist_to_right <= pnc_map.road_half_width / 2.0) {
        return true;
      }
    }
    
    // 如果不接近任何右侧车道中心点，则认为不可行
    return false;
  }
}