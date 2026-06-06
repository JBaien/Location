

aux_source_directory(./ DIR_SRCS)

#################
### Third party
#################

# Find pcap library
find_library(PCAP_LIBRARY pcap)
find_path(PCAP_INCLUDE_DIR pcap.h)
include_directories(${PCAP_INCLUDE_DIR})

add_library(timoo_input SHARED
  ${DIR_SRCS}
) 

target_link_libraries(timoo_input 
  ${PCAP_LIBRARY}
)

# INSTALL(TARGETS tracker LIBRARY DESTINATION lib)

##################################################################################################
## install
##################################################################################################
install(TARGETS timoo_input
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/input_factory.h
  DESTINATION include/input
)

