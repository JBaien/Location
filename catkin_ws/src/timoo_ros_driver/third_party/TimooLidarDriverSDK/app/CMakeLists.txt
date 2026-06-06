

aux_source_directory(. DIR_SRCS)
# aux_source_directory( DIR_SRCS)

# The recommended prefix ensures that target names across packages don't collide
add_library(timoo_sdk_driver SHARED
  timoo_driver.cpp
  timoo_driver_apollo_application.cpp
)

target_link_libraries(timoo_sdk_driver
  timoo_sdk_func
  timoo_sdk_input
  timoo_sdk_data_parser
)



##################################################################################################
## install
##################################################################################################
install(TARGETS timoo_sdk_driver
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DESTINATION include
  FILES_MATCHING 
  PATTERN "*.h"
  PATTERN "*.hpp"
)
