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
  AStar::AStar() // A* 全局规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "A* global planner created.");
    global_planner_config_ = std::make_unique<ConfigReader>();
    global_planner_config_->read_global_path_config();
    global_plannaer_type_ = static_cast<int32_t>(GlobalPlannerType::KASTAR);
  }

  void AStar::CrreateGridMap(const PNCMap &pnc_map, double &map_width, double &map_height, double &centor_x, double &centor_y)
  {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto &point : pnc_map.midline.points)
    {
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

  Path AStar::search_global_path(const PNCMap &pnc_map) // 全局路径搜索
  {
    RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "AStar search_global_path!");

    // 计算起点坐标（道路中线和右边界的中点），不合理，应该是车辆的坐标
    double start_x = (pnc_map.midline.points.front().x + pnc_map.right_boundary.points.front().x) / 2.0;
    double start_y = (pnc_map.midline.points.front().y + pnc_map.right_boundary.points.front().y) / 2.0;
    // 目标点如果是道路终点其实是不合理的，目标点应该是一个指定的坐标（后期修改）
    double goal_x = (pnc_map.midline.points.back().x + pnc_map.right_boundary.points.back().x) / 2.0;
    double goal_y = (pnc_map.midline.points.back().y + pnc_map.right_boundary.points.back().y) / 2.0;

    // 动态重规划机制参数
    const int MAX_RETRY_ATTEMPTS = 3; // 最大重试次数
    const double SCALE_FACTOR = 1.5;  // 每次重试时的地图放大因子
    double map_width = 0.0;
    double map_height = 0.0;
    double centor_x = 0.0;
    double centor_y = 0.0;
    Path final_path;

    CrreateGridMap(pnc_map, map_width, map_height, centor_x, centor_y);
    if (map_width == 0 || map_height == 0)
    {
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
      for (int i = 0; i < midline_size; i++)
      {
        pose_tmp.pose.position.x = (pnc_map.midline.points[i].x + pnc_map.right_boundary.points[i].x) / 2.0;
        pose_tmp.pose.position.y = (pnc_map.midline.points[i].y + pnc_map.right_boundary.points[i].y) / 2.0;
        global_path_.poses.emplace_back(pose_tmp);
      }
      return global_path_;
    }

    // 尝试多次路径规划，每次扩大搜索范围
    for (int attempt = 0; attempt <= MAX_RETRY_ATTEMPTS; ++attempt)
    {
      RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "AStar path planning attempt: %d", attempt);

      // 根据重试次数调整地图尺寸
      double scale = pow(SCALE_FACTOR, attempt);
      const double RESOLUTION = 0.5; // 每个栅格代表0.5米

      // 添加缓冲区确保完全覆盖
      const double GRID_WIDTH = map_width * scale;   // x方向总宽度
      const double GRID_HEIGHT = map_height * scale; // y方向总高度

      // 计算地图原点（确保起点和终点都在地图内）
      double origin_x = centor_x - GRID_WIDTH / 2.0;
      double origin_y = centor_y - GRID_HEIGHT / 2.0;

      // 计算起点和终点在栅格地图中的位置
      int start_grid_x = static_cast<int>((start_x - origin_x) / RESOLUTION);
      int start_grid_y = static_cast<int>((start_y - origin_y) / RESOLUTION);
      int goal_grid_x = static_cast<int>((goal_x - origin_x) / RESOLUTION);
      int goal_grid_y = static_cast<int>((goal_y - origin_y) / RESOLUTION);

      // 检查起点和终点是否在地图范围内
      int grid_width = static_cast<int>(GRID_WIDTH / RESOLUTION);
      int grid_height = static_cast<int>(GRID_HEIGHT / RESOLUTION);

      if (start_grid_x < 0 || start_grid_x >= grid_width || start_grid_y < 0 || start_grid_y >= grid_height)
      {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner_astar.cpp"), "Start point is out of map bounds");
      }

      if (goal_grid_x < 0 || goal_grid_x >= grid_width || goal_grid_y < 0 || goal_grid_y >= grid_height)
      {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner_astar.cpp"), "Goal point is out of map bounds");
      }

      // 初始化开放列表和关闭列表
      auto cmp = [](const Node *a, const Node *b)
      { return a->f > b->f; };
      std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> open_list(cmp);
      std::unordered_set<int> closed_list;
      std::unordered_map<int, Node *> open_list_map;

      // 创建起始节点
      int start_h = static_cast<int>(sqrt(pow(goal_grid_x - start_grid_x, 2) + pow(goal_grid_y - start_grid_y, 2)) * 10);
      Node *start_node = new Node(start_grid_x, start_grid_y, 0, start_h, start_h, nullptr);
      open_list.push(start_node);
      open_list_map[start_grid_x * grid_width + start_grid_y] = start_node;

      // 定义8个方向的移动（包括对角线）
      int dx_move[8] = {0, 1, 1, 1, 0, -1, -1, -1};
      int dy_move[8] = {1, 1, 0, -1, -1, -1, 0, 1};
      int move_cost[8] = {10, 14, 10, 14, 10, 14, 10, 14}; // 直线代价10，对角线代价14

      Node *goal_node = nullptr;

      // A*主循环
      while (!open_list.empty())
      {
        // 取出f值最小的节点
        Node *current = open_list.top();
        open_list.pop();

        // 从开放列表映射中移除
        int current_key = current->x * grid_width + current->y;
        open_list_map.erase(current_key);

        // 将当前节点加入关闭列表
        closed_list.insert(current_key);

        // 检查是否到达目标点
        if (current->x == goal_grid_x && current->y == goal_grid_y)
        {
          goal_node = current;
          break;
        }

        // 探索邻居节点
        for (int i = 0; i < 8; i++)
        {
          int new_x = current->x + dx_move[i];
          int new_y = current->y + dy_move[i];

          // 检查边界
          if (new_x < 0 || new_x >= grid_width || new_y < 0 || new_y >= grid_height)
          {
            continue;
          }
          // 检查是否在关闭列表中
          int new_key = new_x * grid_width + new_y;
          if (closed_list.find(new_key) != closed_list.end())
          {
            continue;
          }
          // 检查是否是可行点（严格限制在道路中线附近的特定点上）
          // 将栅格坐标转换为世界坐标
          double world_x = origin_x + new_x * RESOLUTION;
          double world_y = origin_y + new_y * RESOLUTION;

          // 检查点是否是允许的可行走点
          if (!isPointAllowed(world_x, world_y, pnc_map, RESOLUTION))
          {
            continue;
          }
          // 计算新节点的g、h、f值
          int new_g = current->g + move_cost[i];
          int new_h = static_cast<int>(sqrt(pow(goal_grid_x - new_x, 2) + pow(goal_grid_y - new_y, 2)) * 10);
          int new_f = new_g + new_h;

          // 检查该节点是否已在开放列表中
          auto it = open_list_map.find(new_key);
          if (it != open_list_map.end())
          {
            // 如果已在开放列表中，检查是否找到了更好的路径
            Node *existing_node = it->second;
            if (new_g < existing_node->g)
            {
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
        }
      }

      // 构建路径
      global_path_.poses.clear();
      if (goal_node)
      {
        // 从目标节点回溯到起始节点构建路径
        std::vector<std::pair<double, double>> waypoints;
        Node *current = goal_node;

        while (current != nullptr)
        {
          // 将栅格坐标转换为世界坐标
          double world_x = origin_x + current->x * RESOLUTION;
          double world_y = origin_y + current->y * RESOLUTION;
          waypoints.emplace_back(std::pair{world_x, world_y});
          current = current->parent;
        }

        // 反转路径（因为是从目标回溯到起点）
        std::reverse(waypoints.begin(), waypoints.end());

        // 填充Path消息
        global_path_.header.frame_id = pnc_map.header.frame_id;
        global_path_.header.stamp = rclcpp::Clock().now();

        PoseStamped pose_tmp;
        pose_tmp.header = global_path_.header;
        pose_tmp.pose.orientation.x = 0.0;
        pose_tmp.pose.orientation.y = 0.0;
        pose_tmp.pose.orientation.z = 0.0;
        pose_tmp.pose.orientation.w = 1.0;

        for (const auto &point : waypoints)
        {
          pose_tmp.pose.position.x = point.first;
          pose_tmp.pose.position.y = point.second;
          global_path_.poses.emplace_back(pose_tmp);
        }
        RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "Path found with %zu waypoints", waypoints.size());
        // 成功找到路径，退出循环
        break;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("global_planner_astar.cpp"), "Failed to find a path using A* on attempt %d", attempt);
        // 清理内存
        for (auto &pair : open_list_map)
        {
          delete pair.second;
        }

        // 如果这是最后一次尝试，记录日志
        if (attempt == MAX_RETRY_ATTEMPTS)
        {
          RCLCPP_ERROR(rclcpp::get_logger("global_planner_astar.cpp"), "Failed to find a path after %d attempts", MAX_RETRY_ATTEMPTS + 1);
        }
      }
    }
    // 清理内存（如果还有残留）
    // 注意：这里的清理已经在循环内部处理过了，除非goal_node非空
    // 由于变量作用域的原因，这里不需要额外清理
    RCLCPP_INFO(rclcpp::get_logger("global_planner_astar.cpp"), "AStar global_path created points size: %ld!", global_path_.poses.size());
    return global_path_;
  }

  bool AStar::isPointAllowed(double x, double y, const PNCMap &pnc_map, double resolution)
  {
    // 仅允许在特定的道路上的点移动：
    // 1. 道路中线和右边界的中点（靠右行驶）
    // 2. 道路中线和左边界的中点（用于左转等情况）
    // 3. 道路中线本身（用于通过路口中心区域）
    const auto &midline_points = pnc_map.midline.points;
    const auto &right_boundary_points = pnc_map.right_boundary.points;
    const auto &left_boundary_points = pnc_map.left_boundary.points;

    // 遍历所有道路点，检查给定点是否接近允许的点
    for (size_t i = 0; i < midline_points.size(); ++i)
    {
      // 检查是否接近中线和右边界的中点（正常靠右行驶）
      double right_mid_x = (midline_points[i].x + right_boundary_points[i].x) / 2.0;
      double right_mid_y = (midline_points[i].y + right_boundary_points[i].y) / 2.0;

      // 检查是否接近中线和左边界的中点（左转时可能需要）
      double left_mid_x = (midline_points[i].x + left_boundary_points[i].x) / 2.0;
      double left_mid_y = (midline_points[i].y + left_boundary_points[i].y) / 2.0;

      // 检查是否接近道路中线（通过路口中心区域时需要）
      double center_x = midline_points[i].x;
      double center_y = midline_points[i].y;

      // 检查给定点是否足够接近这些允许的点（在分辨率范围内）
      double dist_to_right = sqrt(pow(x - right_mid_x, 2) + pow(y - right_mid_y, 2));
      double dist_to_left = sqrt(pow(x - left_mid_x, 2) + pow(y - left_mid_y, 2));
      double dist_to_center = sqrt(pow(x - center_x, 2) + pow(y - center_y, 2));

      // 如果点足够接近任何一个允许的点，则认为是可行的
      if (dist_to_right < resolution || dist_to_left < resolution || dist_to_center < resolution)
      {
        return true;
      }
    }
    // 如果不接近任何允许的点，则认为不可行
    return false;
  }
}