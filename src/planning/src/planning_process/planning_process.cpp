#include "planning_process.h"

#include "config_reader.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process") // 规划流程
  {
    RCLCPP_INFO(this->get_logger(), "Planning process created");

    // 读取配置文件
    planning_process_config_ = std::make_unique<ConfigReader>();
    planning_process_config_->read_planning_process_config();
    obs_dis_ = planning_process_config_->process().obs_dis_;

    // 创建车辆和障碍物
    car_ = std::make_unique<MainCar>();
    // 坐标广播器
    tf_broadcaster_ = std::make_unique<StaticTransformBroadcaster>(this);
    // 创建监听器， 绑定主车缓存对象
    buffer_ = std::make_unique<Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);
    // 初始化地图服务和全局路径服务客户端
    pnc_map_client_ = this->create_client<PNCMapService>("pnc_map_service");
    global_path_client_ = this->create_client<GlobalPathService>("global_path_service");
    // 创建参考线和参考线的发布器
    reference_line_creator_ = std::make_unique<ReferenceLineCreator>();
    reference_line_pub_ = this->create_publisher<Path>("reference_line", 10);
  }

  bool PlanningProcess::process() // 总流程
  {
    // 阻塞1s, 等待rviz2和xacro模型启动
    rclcpp::Rate rate(1.0);
    rate.sleep();

    // 流程初始化
    if (!planning_init())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize planning process!");
      return false;
    }

    // 进入规划主流程
    timer_ = this->create_wall_timer(0.1s, std::bind(&PlanningProcess::planning_callback, this)); // 0.1s为周期

    return true;
  }

  bool PlanningProcess::planning_init() // 规划初始化
  {
    // 生成车辆
    vehicle_spawn(car_);

    // 连接地图服务器
    if (!connect_to_server(pnc_map_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to pnc_map_server!");
      return false;
    }

    // 获取地图
    if (!pnc_map_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to request pnc_map!");
      return false;
    }

    // 连接全局路径服务器
    if (!connect_to_server(global_path_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to global_path_server!");
      return false;
    }

    // 获取全局路径
    if (!global_path_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to request global_path!");
      return false;
    }
    return true;
  }

  void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleInfoBase> &vehicle) // 生成车辆
  {
    TransformStamped spawn;
    spawn.header.stamp = this->now();
    spawn.header.frame_id = planning_process_config_->pnc_map().frame_; // 地图坐标
    spawn.child_frame_id = vehicle->child_frame();                      // 地图坐标的子坐标为车辆坐标

    spawn.transform.translation.x = vehicle->local_point().pose.position.x; // 车辆位姿初始化
    spawn.transform.translation.y = vehicle->local_point().pose.position.y;
    spawn.transform.translation.z = vehicle->local_point().pose.position.z;
    spawn.transform.rotation.x = vehicle->local_point().pose.orientation.x; // 四元数初始化
    spawn.transform.rotation.y = vehicle->local_point().pose.orientation.y;
    spawn.transform.rotation.z = vehicle->local_point().pose.orientation.z;
    spawn.transform.rotation.w = vehicle->local_point().pose.orientation.w;

    RCLCPP_INFO(this->get_logger(), "vehicle %s spawned, x = %.2f, y = %.2f",
                spawn.child_frame_id.c_str(), spawn.transform.translation.x, spawn.transform.translation.y);
    tf_broadcaster_->sendTransform(spawn); // 广播出去
  }

  void PlanningProcess::get_location(const std::shared_ptr<VehicleInfoBase> &vehicle) // 获取车辆位置
  {
    try
    {
      PoseStamped point;
      auto ts = buffer_->lookupTransform(planning_process_config_->pnc_map().frame_, vehicle->child_frame(), tf2::TimePointZero);
      point.header = ts.header;
      point.pose.position.x = ts.transform.translation.x;
      point.pose.position.y = ts.transform.translation.y;
      point.pose.position.z = ts.transform.translation.z;
      point.pose.orientation.x = ts.transform.rotation.x;
      point.pose.orientation.y = ts.transform.rotation.y;
      point.pose.orientation.z = ts.transform.rotation.z;
      vehicle->update_location(point);
    }
    catch (const tf2::LookupException &e)
    {
      RCLCPP_ERROR(this->get_logger(), "tf2::LookupException: %s", e.what());
    }
  }

  template <typename T>
  bool PlanningProcess::connect_to_server(const T &client) // 连接服务
  {
    // 服务器名称
    std::string server_name;

    // 判断客户端类型
    if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>) // c++17引入的新特性
    {
      server_name = "pnc_map_server";
    }
    else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
    {
      server_name = "global_path_server";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid client type!");
      return false;
    }

    // 等待服务器连接
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s. Exiting.", server_name.c_str());
        return false;
      }
      RCLCPP_WARN(this->get_logger(), "Waiting for %s to be available...", server_name.c_str());
    }
    return true;
  }

  bool PlanningProcess::pnc_map_request() // pnc地图请求
  {
    RCLCPP_INFO(this->get_logger(), "Requesting pnc_map from pnc_map_server...");
    // 发送请求
    auto request = std::make_shared<PNCMapService::Request>();
    request->map_type = planning_process_config_->pnc_map().type_;

    // 生成响应
    auto result_future = pnc_map_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "pnc_map response success.");
      pnc_map_ = result_future.get()->pnc_map;
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "pnc_map response success failed.");
      return false;
    }
    return false;
  }

  bool PlanningProcess::global_path_request() // 全局路径请求
  {
    RCLCPP_INFO(this->get_logger(), "Requesting global_path from global_path_server...");

    // 发送请求
    auto request = std::make_shared<GlobalPathService::Request>();
    request->pnc_map = pnc_map_;
    request->global_planner_type = planning_process_config_->global_path().type_;

    // 生成响应
    auto result_future = global_path_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "global_path response success.");
      global_path_ = result_future.get()->global_path;
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "global_path response success failed.");
      return false;
    }
    return false;
  }

  void PlanningProcess::planning_callback() // 总流程回调
  {
    // 获取开始时间戳
    const auto start_time = this->get_clock()->now();

    // 监听车辆定位
    get_location(car_);

    // 参考线
    const auto refer_line = reference_line_creator_->create_reference_line(global_path_, car_->local_point());
    if (refer_line.refer_line.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Reference Line empty.");
      return;
    }

    const auto refer_line_rviz = reference_line_creator_->reference_line_to_rviz(); // 显示参考线
    reference_line_pub_->publish(refer_line_rviz);                                  // 发布显示参考线

    // 主车和障碍物向参考线投影

    // 障碍物按s值排序

    // 路径决策

    // 路径规划

    // 障碍物向路径投影

    // 速度决策

    // 速度规划

    // 合成轨迹

    // 更新车辆信息
    RCLCPP_INFO(this->get_logger(), "----car state: loc: (%.2f, %.2f), speed: %.2f, acceleration: %.2f, theta: %.2f, kappa: %.2f,",
                car_->local_point().pose.position.x,
                car_->local_point().pose.position.y,
                car_->speed(), car_->acceleration(),
                car_->theta(), car_->kappa());

    const auto end_time = this->get_clock()->now();
    const double planning_total_time = (end_time - start_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Planning total time: %fms\n", planning_total_time * 1000);

    // 防止卡死
    if (planning_total_time > 1.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Planning total time is too long: %fms\n", planning_total_time * 1000);
      rclcpp::shutdown();
    }
  }

} // namespace Planning