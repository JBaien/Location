# build the driver node
add_executable(lidar_node lidar_node.cc driver.cc)
add_dependencies(lidar_node lidar_driver_gencfg)
target_link_libraries(lidar_node
  lidar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
add_dependencies(driver_nodelet lidar_driver_gencfg)
target_link_libraries(driver_nodelet
  lidar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS lidar_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
