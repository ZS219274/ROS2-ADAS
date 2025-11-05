#include "global_path_server.h"

namespace Planning
{
  GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器节点
  {
    RCLCPP_INFO(this->get_logger(), "Global path server start!");
    //初始化发布器for planning
    global_path_pub_ = this->create_publisher<Path>("global_path", 10);

    //初始化发布器for rviz
    global_path_rviz_pub_ = this->create_publisher<Marker>("pnc_map_rviz", 10);

    //初始化服务端
    global_path_server_ = this->create_service<GlobalPathService>(
        "global_path_server",
        std::bind(&GlobalPathServer::response_global_path_callback, this, _1, _2));
  }

  // 全局路径回调
  void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                       std::shared_ptr<GlobalPathService::Response> response)
  {
    //接收请求，多态
    switch (request->global_planner_type)
    {
    case static_cast<int32_t>(GlobalPlannerType::KNORMAL):
    {
      global_planner_creator_ = std::make_shared<GlobalPlannerNormal>();
    }
    break;
    default:
      RCLCPP_WARN(this->get_logger(), "Invalid Global planner type!");
      return;
    }

    //判断请求是否为空
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
    (void)path;
    Marker path_rviz;
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