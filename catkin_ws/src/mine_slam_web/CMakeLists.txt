cmake_minimum_required(VERSION 3.0.2)
project(mine_slam_web)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  target_localizer
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenSSL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs target_localizer visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

add_library(mine_slam_web_lib
  src/mesh_encoder.cpp
  src/pointcloud_downsampler.cpp
  src/pointcloud_encoder.cpp
  src/websocket_server.cpp
)
add_dependencies(mine_slam_web_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mine_slam_web_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenSSL::Crypto
)

add_executable(mine_web_bridge_node src/mine_web_bridge_node.cpp)
add_dependencies(mine_web_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mine_web_bridge_node
  mine_slam_web_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mesh_encoder_test test/mesh_encoder_test.cpp)
  if(TARGET mesh_encoder_test)
    target_link_libraries(mesh_encoder_test
      mine_slam_web_lib
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS mine_slam_web_lib mine_web_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
