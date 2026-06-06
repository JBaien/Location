
cmake_minimum_required(VERSION 3.5)

project(TimooLidarDriverSDK
  VERSION 0.0.6
  DESCRIPTION "Timoo software for lidar driver" 
  LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_BUILD_TYPE  Release) # Release, Debug
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET( CMAKE_CXX_FLAGS "-O3")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX
      "/opt/timoo_lidar_driver/"
      CACHE PATH "My default install prefix" FORCE)
endif()
add_definitions(-DCMAKE_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\")

include(cmake/timoo_utils.cmake)
include(CMakePackageConfigHelpers)
include(CMakeDependentOption)
include(CheckCXXCompilerFlag)
include(GNUInstallDirs)
if(NOT PACKAGE_HUB)
  timoo_make_package(${PROJECT_NAME} ${PROJECT_VERSION} ${PROJECT_DESCRIPTION})
endif()


#################
### Third party
#################
# Find pcap library
find_library(PCAP_LIBRARY pcap)
find_path(PCAP_INCLUDE_DIR pcap.h)
include_directories(${PCAP_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR})

###########
## Build ##
###########

# Add library
add_subdirectory(./common)
add_subdirectory(./interface)
add_subdirectory(./input)
add_subdirectory(./data_parser)
add_subdirectory(./app)
add_subdirectory(./func)

set(SUB_LIBS
  timoo_sdk_input
  timoo_sdk_data_parser
  timoo_sdk_driver
  timoo_sdk_func
)

# 创建一个SDK接口库，用于整合所有子模块
add_library(${PROJECT_NAME} INTERFACE)

# 链接所有子模块库到接口库
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${SUB_LIBS}
  ${PCAP_LIBRARY}
)

# 导出接口库的包含目录
target_include_directories(${PROJECT_NAME} INTERFACE
  ${PROJECT_SOURCE_DIR}
  ${PCAP_INCLUDE_DIR}
)


if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  ##################################################################################################
  ## cpack
  ##################################################################################################
  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in"
    "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}"
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR)

  write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}-config-version.cmake"
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion)

  install(FILES "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
                "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}-config-version.cmake"
          DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}")
endif()
