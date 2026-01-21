# 查找第三方库配置文件
# 设置第三方库路径
# 从 src/planning 到项目根目录: ../../third_party
get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)  # src/planning -> src
get_filename_component(PROJECT_ROOT ${PROJECT_ROOT} DIRECTORY)  # src -> ROS2-ADAS
set(THIRD_PARTY_ROOT ${PROJECT_ROOT}/third_party)
set(THIRD_PARTY_INCLUDE_DIR ${THIRD_PARTY_ROOT}/include)
set(THIRD_PARTY_LIB_DIR ${THIRD_PARTY_ROOT}/lib)

# Eigen3 配置
set(EIGEN3_INCLUDE_DIR ${THIRD_PARTY_INCLUDE_DIR}/eigen-3.4.0)
if(EXISTS ${EIGEN3_INCLUDE_DIR}/Eigen)
  set(EIGEN3_FOUND TRUE)
  message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
else()
  set(EIGEN3_FOUND FALSE)
  message(FATAL_ERROR "Eigen3 not found in ${EIGEN3_INCLUDE_DIR}")
endif()

# 创建 Eigen3::Eigen 接口库（header-only）
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
  )
endif()

# OsqpEigen 配置
set(OSQPEIGEN_INCLUDE_DIR ${THIRD_PARTY_INCLUDE_DIR}/OsqpEigen)
if(EXISTS ${OSQPEIGEN_INCLUDE_DIR}/OsqpEigen.h)
  set(OSQPEIGEN_FOUND TRUE)
  message(STATUS "Found OsqpEigen: ${OSQPEIGEN_INCLUDE_DIR}")
else()
  set(OSQPEIGEN_FOUND FALSE)
  message(FATAL_ERROR "OsqpEigen not found in ${OSQPEIGEN_INCLUDE_DIR}")
endif()

# OSQP 库配置
set(OSQP_INCLUDE_DIR ${THIRD_PARTY_INCLUDE_DIR}/osqp)
set(OSQP_LIB_DIR ${THIRD_PARTY_LIB_DIR})

# 查找 OSQP 库文件
find_library(OSQP_LIBRARY
  NAMES osqp
  PATHS ${OSQP_LIB_DIR}
  NO_DEFAULT_PATH
)

find_library(OSQP_STATIC_LIBRARY
  NAMES osqpstatic
  PATHS ${OSQP_LIB_DIR}
  NO_DEFAULT_PATH
)

if(OSQP_LIBRARY OR OSQP_STATIC_LIBRARY)
  set(OSQP_FOUND TRUE)
  message(STATUS "Found OSQP library: ${OSQP_LIB_DIR}")
else()
  set(OSQP_FOUND FALSE)
  message(FATAL_ERROR "OSQP library not found in ${OSQP_LIB_DIR}")
endif()

# 查找 OsqpEigen 库文件
find_library(OSQPEIGEN_LIBRARY
  NAMES OsqpEigen
  PATHS ${OSQP_LIB_DIR}
  NO_DEFAULT_PATH
)

if(OSQPEIGEN_LIBRARY)
  set(OSQPEIGEN_LIB_FOUND TRUE)
  message(STATUS "Found OsqpEigen library: ${OSQPEIGEN_LIBRARY}")
else()
  set(OSQPEIGEN_LIB_FOUND FALSE)
  message(WARNING "OsqpEigen library not found in ${OSQP_LIB_DIR}, using header-only version")
endif()

# 创建 OsqpEigen::OsqpEigen 接口库
if(NOT TARGET OsqpEigen::OsqpEigen)
  add_library(OsqpEigen::OsqpEigen INTERFACE IMPORTED)
  # 使用 target_include_directories 并设置 BEFORE 以确保优先级
  # OSQP 必须在 OsqpEigen 之前，以确保类型定义正确
  target_include_directories(OsqpEigen::OsqpEigen INTERFACE
    $<BUILD_INTERFACE:${OSQP_INCLUDE_DIR}>      # OSQP 先
    $<BUILD_INTERFACE:${OSQPEIGEN_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  )
  # 定义宏以确保使用正确的 OSQP 版本
  target_compile_definitions(OsqpEigen::OsqpEigen INTERFACE OSQP_EIGEN_OSQP_IS_V1)
  set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
    INTERFACE_LINK_DIRECTORIES "${OSQP_LIB_DIR}"
  )
  # 链接 OSQP 和 OsqpEigen 库
  if(OSQPEIGEN_LIBRARY)
    set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
      INTERFACE_LINK_LIBRARIES "${OSQPEIGEN_LIBRARY};${OSQP_LIBRARY}"
    )
  else()
    set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
      INTERFACE_LINK_LIBRARIES "${OSQP_LIBRARY}"
    )
  endif()
endif()

# 导出变量供其他 CMakeLists.txt 使用
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(OSQPEIGEN_INCLUDE_DIRS ${OSQPEIGEN_INCLUDE_DIR})
set(OSQP_INCLUDE_DIRS ${OSQP_INCLUDE_DIR})
set(OSQP_LIBRARIES ${OSQP_LIBRARY})
# 设置为缓存变量，以便子目录访问
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen3 include directory")
set(OSQPEIGEN_INCLUDE_DIR ${OSQPEIGEN_INCLUDE_DIR} CACHE PATH "OsqpEigen include directory")
set(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_DIR} CACHE PATH "OSQP include directory")

