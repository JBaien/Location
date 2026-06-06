

aux_source_directory(. DIR_SRCS)
# aux_source_directory( DIR_SRCS)

# The recommended prefix ensures that target names across packages don't collide
add_library(timoo_func SHARED
    advanced_setting.cpp
)




##################################################################################################
## install
##################################################################################################
install(TARGETS timoo_func
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
