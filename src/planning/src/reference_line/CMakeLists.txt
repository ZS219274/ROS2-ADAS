cmake_minimum_required(VERSION 3.22)
project(reference_line)

#设置依赖
set(DEPENDS 
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
)

#添加动态库
add_library(reference_line SHARED 
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_creator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_smoother.cpp
)

set(target
  reference_line
)

#添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

# 确保使用项目中的第三方库，优先于系统路径
# 重新计算路径（从当前目录到项目根目录）
# src/planning/src/reference_line -> src/planning/src -> src/planning -> src -> 项目根目录
get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)  # src/reference_line -> src
get_filename_component(PROJECT_ROOT ${PROJECT_ROOT} DIRECTORY)  # src -> planning
get_filename_component(PROJECT_ROOT ${PROJECT_ROOT} DIRECTORY)  # planning -> src
get_filename_component(PROJECT_ROOT ${PROJECT_ROOT} DIRECTORY)  # src -> 项目根目录
set(THIRD_PARTY_INCLUDE_DIR ${PROJECT_ROOT}/third_party/include)

# 使用 BEFORE 确保优先级最高
# 注意：应该包含到 third_party/include 目录，而不是子目录
target_include_directories(${target} BEFORE PRIVATE
  ${THIRD_PARTY_INCLUDE_DIR}/eigen-3.4.0
  ${THIRD_PARTY_INCLUDE_DIR}/osqp      # OSQP 先
  ${THIRD_PARTY_INCLUDE_DIR}           # OsqpEigen 的父目录
)
# 明确排除系统路径中的 OsqpEigen
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${target} PRIVATE
    -idirafter /usr/local/include  # 将系统路径移到后面
  )
endif()
target_compile_definitions(${target} PRIVATE OSQP_EIGEN_OSQP_IS_V1)

target_link_libraries(${target}
  config_reader
  mathlibs
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

#安装动态库、节点和相关目录
install (TARGETS 
  ${target}
  LIBRARY DESTINATION lib)

