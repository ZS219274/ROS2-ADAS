#include "global_path_server.h"

namespace Planning
{
  GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器节点
  {
    RCLCPP_INFO(this->get_logger(), "Global path server start!");
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