cmake_minimum_required(VERSION 3.0.2)
project(timoo_ros_driver VERSION 0.0.1 LANGUAGES CXX)

#=======================================
# 项目描述：Timoo激光雷达的ROS驱动程序
# 支持点云类型：XYZI, XYZIRT
#=======================================

# 设置点云类型 (XYZI 或 XYZIRT)
set(POINT_TYPE XYZIRT CACHE STRING "Point cloud type: XYZI or XYZIRT")
set_property(CACHE POINT_TYPE PROPERTY STRINGS XYZI XYZIRT)

# C++ 标准设置
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# 编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type: Release or Debug" FORCE)
endif()

# 编译优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -Wall -Wextra)
else()
  add_compile_options(-g -Wall -Wextra)
endif()

#========================
# 点云类型定义
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
  message(STATUS "Point type is set to XYZI")
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
  message(STATUS "Point type is set to XYZIRT")
else()
  message(FATAL_ERROR "Unsupported point type: ${POINT_TYPE}. Choose XYZI or XYZIRT.")
endif()

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
)

## 查找第三方库

# 查找PCL库
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########
## 编译SDK ##
###########

# 添加TimooLidarDriverSDK作为子目录
# 假设SDK位于third_party目录下，根据实际路径调整
add_subdirectory(third_party/TimooLidarDriverSDK)

###########
## Build ##
###########

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # 添加SDK的头文件目录
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/TimooLidarDriverSDK
)

# 生成可执行文件
add_executable(${PROJECT_NAME}_node 
  src/main.cpp
)

# 添加依赖，确保SDK先于主程序编译
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  TimooLidarDriverSDK  # 依赖SDK目标
)

# 链接库，包括SDK生成的库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  TimooLidarDriverSDK  # 链接SDK库
)


# --------------------------
# 添加调试打印信息
# --------------------------
# 确保TimooLidarDriverSDK目标已定义
# 在添加SDK子目录之后，链接目标之前添加以下代码

# if(TARGET TimooLidarDriverSDK)
#   message(STATUS "--------------------------------------------------")
#   message(STATUS "TimooLidarDriverSDK 包含的库:")
  
#   get_target_property(SDK_LINK_LIBS TimooLidarDriverSDK INTERFACE_LINK_LIBRARIES)
  
#   if(SDK_LINK_LIBS)
#     foreach(lib ${SDK_LINK_LIBS})
#       message(STATUS "  - ${lib}")
      
#       # 只检查目标是否存在，不获取路径
#       if(TARGET ${lib})
#         message(STATUS "    状态: 有效目标（将在构建时生成）")
#       else()
#         message(STATUS "    状态: 非目标库（可能是系统库）")
#       endif()
#     endforeach()
#   else()
#     message(WARNING "  TimooLidarDriverSDK没有链接任何库")
#   endif()
  
#   message(STATUS "--------------------------------------------------")
# else()
#   message(WARNING "TimooLidarDriverSDK目标不存在!")
# endif()
#############
## Install ##
#############

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装启动文件（如果有的话）
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
