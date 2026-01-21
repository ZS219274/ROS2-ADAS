cmake_minimum_required(VERSION 3.22)

#设置依赖
set(DEPENDS 
  "rclcpp"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "base_msgs"
  "planning"
)

#添加节点
add_executable(control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/control_node.cpp
)

#添加依赖
ament_target_dependencies(control_node
  ${DEPENDS}
)

# 查找 planning 模块的库路径
find_library(VEHICLE_INFO_LIB
  NAMES vehicle_info
  PATHS ${CMAKE_INSTALL_PREFIX}/planning/lib
        ${CMAKE_INSTALL_PREFIX}/../install/planning/lib
        /home/ubuntu/wxx/project/ROS2-ADAS/install/planning/lib
  NO_DEFAULT_PATH
)

target_link_libraries(control_node
  config_reader
  pid_controller
  mpc_controller
  lqr_controller
  ${VEHICLE_INFO_LIB}
  yaml-cpp
)

#安装节点
install(TARGETS 
  control_node
  DESTINATION lib/${CONTROL_PACKAGE_NAME})

