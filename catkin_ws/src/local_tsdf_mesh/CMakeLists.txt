cmake_minimum_required(VERSION 3.0.2)
project(local_tsdf_mesh)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
)
find_package(PCL 1.7 REQUIRED COMPONENTS common filters kdtree registration)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_tsdf_mesh_core
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    visualization_msgs
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(local_tsdf_mesh_core
  src/frame_registrar.cpp
  src/marker_points.cpp
  src/mesh_cleanup.cpp
  src/scan_strip_support.cpp
  src/sparse_tsdf.cpp
)
add_dependencies(local_tsdf_mesh_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_tsdf_mesh_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(local_tsdf_mesh_node src/local_tsdf_mesh_node.cpp)
add_dependencies(local_tsdf_mesh_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_tsdf_mesh_node
  local_tsdf_mesh_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(local_tsdf_mesh_core_test
    test/local_tsdf_mesh_core_test.cpp
  )
  if(TARGET local_tsdf_mesh_core_test)
    target_link_libraries(local_tsdf_mesh_core_test
      local_tsdf_mesh_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(local_tsdf_mesh_cleanup_test
    test/mesh_cleanup_test.cpp
  )
  if(TARGET local_tsdf_mesh_cleanup_test)
    target_link_libraries(local_tsdf_mesh_cleanup_test
      local_tsdf_mesh_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(local_tsdf_scan_strip_support_test
    test/scan_strip_support_test.cpp
  )
  if(TARGET local_tsdf_scan_strip_support_test)
    target_link_libraries(local_tsdf_scan_strip_support_test
      local_tsdf_mesh_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(local_tsdf_marker_points_test
    test/marker_points_test.cpp
  )
  if(TARGET local_tsdf_marker_points_test)
    target_link_libraries(local_tsdf_marker_points_test
      local_tsdf_mesh_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
endif()

install(TARGETS local_tsdf_mesh_core local_tsdf_mesh_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
