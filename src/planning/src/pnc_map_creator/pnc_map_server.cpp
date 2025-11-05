#include "pnc_map_server.h"

namespace Planning
{

  PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "PNCMapServer created.");
    // 地图发布器
    pnc_map_pub_ = this->create_publisher<PNCMap>("pnc_map", 10);
    pnc_map_rviz_pub_ = this->create_publisher<MarkerArray>("pnc_map_markarray", 10);

    // 地图服务器
    pnc_map_server_ = this->create_service<PNCMapService>(
        "pnc_map_server",
        std::bind(&PNCMapServer::response_pnc_map_callback, this, _1, _2));
  }

  // 响应并发布地图
  void PNCMapServer::response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request,
                                               std::shared_ptr<PNCMapService::Response> response)
  {
    // 接收请求，  多态
    switch (request->map_type)
    {
    case static_cast<int32_t>(PncMapType::KSTRAIGHT):
      pnc_map_creator_ = std::make_shared<Planning::PncMapCreatorStraight>();
      break;
    case static_cast<int32_t>(PncMapType::kSTURN):
      pnc_map_creator_ = std::make_shared<Planning::PncMapCreatorSturn>();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Invalid pnc_map type!");
      return;
    }

    // 创建并响应地图
    const auto pnc_map = pnc_map_creator_->create_pnc_map();
    response->pnc_map = std::move(pnc_map);

    // 发布地图  planning
    pnc_map_pub_->publish(response->pnc_map);
    RCLCPP_INFO(this->get_logger(), "pnc_map for planning published.");

    // 发布地图  rviz
    const auto pnc_map_mark_array = pnc_map_creator_->pnc_map_mark_array();
    pnc_map_rviz_pub_->publish(pnc_map_mark_array);
    RCLCPP_INFO(this->get_logger(), "pnc_map for rviz published.");
  }
} // namespace Planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::PNCMapServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}