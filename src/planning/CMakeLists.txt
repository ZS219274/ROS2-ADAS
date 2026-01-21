cmake_minimum_required(VERSION 3.22)
project(planning)

# 设置C和C++标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMAKE_COMPILER_IS_GNUCXX在新版cmake中已经弃用,改用CMAKE_CXX_COMPILER_ID
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置预编译指令, 生成compile_commands.json
# 然后让c_cpp_properties.json读取这个文件, 防止预编译找不到cmake中定义的宏而报错
# c_cpp_properties.json中加上"compileCommands": "${workspaceFolder}/build/compile_commands.json"来读取这个文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置包名, 让子目录节点安装时使用
set(PLANNING_PACKAGE_NAME ${PROJECT_NAME})

# 加载第三方库配置
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindThirdParty.cmake)

# 确保优先使用项目中的第三方库，而不是系统路径
# 使用 -I 选项明确指定包含路径优先级（在系统路径之前）
# OSQP 必须在 OsqpEigen 之前
# 注意：OsqpEigen 应该包含其父目录 third_party/include
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-I${EIGEN3_INCLUDE_DIR})
  add_compile_options(-I${OSQP_INCLUDE_DIR})      # OSQP 先于 OsqpEigen
  get_filename_component(THIRD_PARTY_INCLUDE_DIR ${OSQPEIGEN_INCLUDE_DIR} DIRECTORY)
  add_compile_options(-I${THIRD_PARTY_INCLUDE_DIR})  # OsqpEigen 的父目录
endif()

# 定义宏以确保 OsqpEigen 使用正确的 OSQP 版本
add_definitions(-DOSQP_EIGEN_OSQP_IS_V1)

# 包含头文件目录（BEFORE 确保第三方库路径优先于系统路径）
# OSQP 必须在 OsqpEigen 之前，以确保类型定义正确
# 注意：OsqpEigen 应该包含其父目录 third_party/include
get_filename_component(THIRD_PARTY_INCLUDE_DIR ${OSQPEIGEN_INCLUDE_DIR} DIRECTORY)
include_directories(BEFORE
  ${EIGEN3_INCLUDE_DIR}
  ${OSQP_INCLUDE_DIR}      # OSQP 先
  ${THIRD_PARTY_INCLUDE_DIR}  # OsqpEigen 的父目录
)
include_directories(
  src/common/config_reader
  src/common/math
  src/decision_center
  src/pnc_map_creator
  src/pnc_map_creator/pnc_map_straight
  src/pnc_map_creator/pnc_map_sturn
  src/global_planner
  src/global_planner/global_planner_normal
  src/local_planner
  src/local_planner/local_path
  src/local_planner/local_speeds
  src/reference_line
  src/test/osqp_test
  src/vehicle_info
  src/vehicle_info/main_car
  src/vehicle_info/obs_car
  src/move_cmd
)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(base_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Eigen3 和 OsqpEigen 已通过 FindThirdParty.cmake 配置，不再使用系统安装的版本
# find_package(Eigen3 REQUIRED)
# find_package(OsqpEigen REQUIRED)

# 添加子目录
add_subdirectory(src/common)
add_subdirectory(src/decision_center)
add_subdirectory(src/pnc_map_creator)
add_subdirectory(src/global_planner)
add_subdirectory(src/local_planner)
add_subdirectory(src/reference_line)
add_subdirectory(src/test)
add_subdirectory(src/vehicle_info)
add_subdirectory(src/planning_process)
add_subdirectory(src/move_cmd)

# 安装辅助目录
install(DIRECTORY
  config launch urdf rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
