cmake_minimum_required(VERSION 3.0.2)
project(target_localizer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  EquipmentState.msg
  TargetMeasurement.msg
  TargetXY.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES target_localizer_core
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_runtime
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(target_localizer_core
  src/cylinder_geometry.cpp
  src/equipment_geometry.cpp
  src/target_tracker.cpp
)
add_dependencies(target_localizer_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_localizer_core
  ${catkin_LIBRARIES}
)

add_executable(target_localizer_node
  src/target_localizer_node.cpp
)
add_dependencies(target_localizer_node
  target_localizer_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_localizer_node
  target_localizer_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(equipment_state_node
  src/equipment_state_node.cpp
)
add_dependencies(equipment_state_node
  target_localizer_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(equipment_state_node
  target_localizer_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(modbus_sensor_reference_node
  src/modbus_sensor_reference_node.cpp
)
add_dependencies(modbus_sensor_reference_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(modbus_sensor_reference_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS target_localizer_core target_localizer_node equipment_state_node modbus_sensor_reference_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(target_localizer_core_test
    test/target_localizer_core_test.cpp
  )
  target_link_libraries(target_localizer_core_test
    target_localizer_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
endif()
