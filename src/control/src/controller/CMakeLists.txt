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
# 优先查找 colcon 工作空间的 install 目录
# 获取工作空间根目录（从 src/control/src/controller 向上到工作空间根目录）
get_filename_component(WORKSPACE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../ ABSOLUTE)
set(INSTALL_DIR ${WORKSPACE_ROOT}/install)

find_library(VEHICLE_INFO_LIB
  NAMES vehicle_info
  PATHS
    ${INSTALL_DIR}/planning/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../install/planning/lib
    ${CMAKE_INSTALL_PREFIX}/planning/lib
    $ENV{COLCON_PREFIX_PATH}/planning/lib
  PATH_SUFFIXES
    lib
  NO_DEFAULT_PATH
)

# 如果上面的路径找不到，尝试使用默认路径（适用于已安装的包）
if(NOT VEHICLE_INFO_LIB)
  find_library(VEHICLE_INFO_LIB
    NAMES vehicle_info
    PATHS
      ${CMAKE_PREFIX_PATH}/planning/lib
    PATH_SUFFIXES
      lib
  )
endif()

# 如果仍然找不到，尝试直接链接目标（适用于同一工作空间构建时）
if(NOT VEHICLE_INFO_LIB)
  # 检查 vehicle_info 目标是否存在（在同一工作空间构建时）
  if(TARGET vehicle_info)
    set(VEHICLE_INFO_LIB vehicle_info)
  else()
    message(FATAL_ERROR "Could not find vehicle_info library. Please ensure planning package is built before control package.")
  endif()
endif()

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