cmake_minimum_required(VERSION 3.22)

# 添加库
add_library(config_reader
  config_reader/config_reader.cpp
)

add_library(pid_controller
  pid_controller/pid_controller.cpp
)

add_library(mpc_controller
  mpc_controller/mpc_controller.cpp
)

add_library(lqr_controller
  lqr_controller/lqr_controller.cpp
)

ament_target_dependencies(config_reader
  rclcpp
  yaml-cpp
  ament_index_cpp
)

ament_target_dependencies(pid_controller
  rclcpp
  tf2
  base_msgs
)

ament_target_dependencies(mpc_controller
  rclcpp
  tf2
  base_msgs
)

ament_target_dependencies(lqr_controller
  rclcpp
  tf2
  base_msgs
)

# 安装库
install(TARGETS
  config_reader
  pid_controller
  mpc_controller
  lqr_controller
  DESTINATION lib/${CONTROL_PACKAGE_NAME}
)

# 安装头文件
install(DIRECTORY
  config_reader/
  pid_controller/
  mpc_controller/
  lqr_controller/
  controller_base/
  DESTINATION include/${CONTROL_PACKAGE_NAME}
  FILES_MATCHING PATTERN "*.h"
)

