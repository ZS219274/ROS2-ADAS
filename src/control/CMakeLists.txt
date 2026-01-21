cmake_minimum_required(VERSION 3.22)
project(control)

# 设置C和C++标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 设置预编译指令, 生成compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置包名
set(CONTROL_PACKAGE_NAME ${PROJECT_NAME})

# 包含头文件目录（planning 模块的路径必须在 control 模块之前，避免命名冲突）
include_directories(
  # planning 模块的头文件路径（必须先包含，确保 Planning::ConfigReader 先被找到）
  ${CMAKE_CURRENT_SOURCE_DIR}/../planning/src/common/config_reader
  ${CMAKE_CURRENT_SOURCE_DIR}/../planning/src/common/math
  ${CMAKE_CURRENT_SOURCE_DIR}/../planning/src/vehicle_info
  ${CMAKE_CURRENT_SOURCE_DIR}/../planning/src/vehicle_info/main_car
  ${CMAKE_CURRENT_SOURCE_DIR}/../planning/src/vehicle_info/obs_car
  # control 模块的头文件路径
  src/common/config_reader
  src/common/pid_controller
  src/common/controller_base
  src/common/mpc_controller
  src/common/lqr_controller
  src/controller
)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(base_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(planning REQUIRED)

# 添加子目录
add_subdirectory(src/common)
add_subdirectory(src/controller)

# 安装辅助目录
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

