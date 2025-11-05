#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "pnc_map_creator_straight.h"
#include "pnc_map_creator_sturn.h"

namespace Planning
{
  using base_msgs::srv::PNCMapService;
  using std::placeholders:: _1;
  using std::placeholders:: _2;

  class PNCMapServer : public rclcpp::Node
  {
  public:
    PNCMapServer();

  private:
    // 响应并发布地图的回调
    void response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request,
                                   std::shared_ptr<PNCMapService::Response> response);

  private:
    std::shared_ptr<Planning::PncMapCreatorBase> pnc_map_creator_; // pnc地图创建器
    rclcpp::Publisher<PNCMap>::SharedPtr pnc_map_pub_;             // 地图发布器
    rclcpp::Publisher<MarkerArray>::SharedPtr pnc_map_rviz_pub_;   // 地图MarkerArray发布器
    rclcpp::Service<PNCMapService>::SharedPtr pnc_map_server_;    // pnc地图服务器
  };

} // namespace Planning
#endif // PNC_MAP_SERVER_H_