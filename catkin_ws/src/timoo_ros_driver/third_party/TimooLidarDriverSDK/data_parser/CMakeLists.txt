

aux_source_directory(./ DIR_SRCS)
aux_source_directory(./tm16 DIR_SRCS)
aux_source_directory(./tm32 DIR_SRCS)
aux_source_directory(./tm1550_standard DIR_SRCS)
aux_source_directory(./tm128 DIR_SRCS)
aux_source_directory(./tm128v DIR_SRCS)

# add_definitions(-DHAVE_NEW_YAMLCPP)

add_library(timoo_sdk_data_parser SHARED
  ${DIR_SRCS}
) 

# target_link_libraries(timoo_sdk_data_parser
#   yaml-cpp
# )

# INSTALL(TARGETS tracker LIBRARY DESTINATION lib)

##################################################################################################
## install
##################################################################################################
install(TARGETS timoo_sdk_data_parser
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/parser_factory.h
  DESTINATION include/data_parser
)

