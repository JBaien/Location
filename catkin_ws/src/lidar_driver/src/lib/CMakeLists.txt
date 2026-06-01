add_library(lidar_input input.cc)
target_link_libraries(lidar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(lidar_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS lidar_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
