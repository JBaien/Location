cmake_minimum_required(VERSION 3.0.2)
project(lidar_fusion)

## 添加编译标准
add_compile_options(-std=c++14)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  message_filters
)

## 查找其他依赖
find_package(PCL REQUIRED)
# Eigen3通常已经包含在PCL中，不需要单独查找

## 系统依赖
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## catkin包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_fusion
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs 
    tf2_eigen 
    pcl_ros 
    pcl_conversions 
    message_filters
  DEPENDS 
    PCL 
    Boost
)

###########
## Build ##
###########

## 指定包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## 指定链接目录
link_directories(${PCL_LIBRARY_DIRS})

## 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

## 声明C++库
add_library(${PROJECT_NAME}
  src/lidar_fusion.cpp
  src/multi_lidar_fusion.cpp
)

## 添加cmake目标依赖，确保在编译这个库之前消息被生成
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定库的链接
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

## 声明C++可执行文件
add_executable(${PROJECT_NAME}_node src/lidar_fusion_node.cpp)
add_executable(multi_lidar_fusion_node src/multi_lidar_fusion_node.cpp)

## 重命名C++可执行文件（可选）
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME lidar_fusion_node PREFIX "")

## 添加cmake目标依赖，确保在编译这个可执行文件之前消息和动态重配置文件被生成
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(multi_lidar_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定可执行文件的链接库
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(multi_lidar_fusion_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# 标记要安装的可执行文件和/或库
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node multi_lidar_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 标记要安装的C++头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## 标记要安装的其他文件（如launch和配置文件）
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

#############
## Testing ##
#############

## 添加gtest基础的C++测试
# catkin_add_gtest(${PROJECT_NAME}-test src/test_lidar_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加基于文件夹的Python doctest
# catkin_add_nosetests(test)
