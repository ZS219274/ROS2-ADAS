#include "global_path_server.h"

namespace Planning
{
  GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器节点
  {
    RCLCPP_INFO(this->get_logger(), "Global path server start!");
    // 初始化发布器for planning
    global_path_pub_ = this->create_publisher<Path>("global_path", 10);

    // 初始化发布器for rviz
    global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz", 10);

    // 初始化服务端
    global_path_server_ = this->create_service<GlobalPathService>(
        "global_path_service",
        std::bind(&GlobalPathServer::response_global_path_callback, this, _1, _2));
  }

  // 全局路径回调
  void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                       std::shared_ptr<GlobalPathService::Response> response)
  {
    // 接收请求，多态
    switch (request->global_planner_type)
    {
    case static_cast<int32_t>(GlobalPlannerType::KNORMAL):
    {
      global_planner_creator_ = std::make_shared<GlobalPlannerNormal>();
    }
    break;
    case static_cast<int32_t>(GlobalPlannerType::KASTAR):
    {
      global_planner_creator_ = std::make_shared<AStar>();
    }
    break;
    default:
      RCLCPP_WARN(this->get_logger(), "Invalid Global planner type!");
      return;
    }

    // 判断请求是否为空
    if (request->pnc_map.midline.points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "pnc_map is empty, global path can not be created!");
      return;
    }

    const auto global_path = global_planner_creator_->search_global_path(request->pnc_map);
    response->global_path = std::move(global_path);
    global_path_pub_->publish(response->global_path);
    RCLCPP_INFO(this->get_logger(), "global_path for planning published!");

    const auto path_rviz = Path2Marker(response->global_path);
    global_path_rviz_pub_->publish(path_rviz);
    RCLCPP_INFO(this->get_logger(), "global_path for rviz published!");
  }
  // path转Marker
  Marker GlobalPathServer::Path2Marker(const Path &path)
  {
    Marker path_rviz;
    path_rviz.header = path.header;
    path_rviz.ns = "global_path";
    path_rviz.id = 0;
    path_rviz.type = Marker::LINE_STRIP;          // 连续线条
    path_rviz.action = Marker::ADD;               // 不断增长
    path_rviz.scale.x = 0.05;                     // 线宽
    path_rviz.color.a = 1.0;                      // 透明度
    path_rviz.color.r = 0.8;                      // 红色
    path_rviz.color.g = 0.0;                      // 绿色
    path_rviz.color.b = 0.0;                      // 蓝色
    path_rviz.lifetime = rclcpp::Duration::max(); // 无限生命期
    path_rviz.frame_locked = true;                // 锁定坐标系

    Point p_tmp;
    for (const auto &pose : path.poses)
    {
      p_tmp.x = pose.pose.position.x;
      p_tmp.y = pose.pose.position.y;
      path_rviz.points.emplace_back(p_tmp);
    }
    return path_rviz;
  }

} // namespace Planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::GlobalPathServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}