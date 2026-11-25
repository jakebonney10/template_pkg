cmake_minimum_required(VERSION 3.16)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
    ament_cmake
    rclcpp
    std_msgs
    diagnostic_updater
    template_pkg_interfaces
  )

set(EXTERNAL_DEPENDS

  )

set(EXTERNAL_DEPENDS_LIB

  )


get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in


add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for ROS2 Humble and Iron
# Use C++20 for ROS2 Jazzy and newer distributions
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)


################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

include_directories(
  include
)

###########
## Build ##
###########

# Collect the imported targets of every ROS dependency.  This replaces
# ament_target_dependencies(), which was removed in ROS2 Lyrical, and works
# on Humble and newer.  Message packages export <pkg>_TARGETS, most other
# packages export a <pkg>::<pkg> target.
set(ROS_DEPENDS_LIB "")
foreach(DEPEND ${ROS_DEPENDS})
  if(DEFINED ${DEPEND}_TARGETS)
    list(APPEND ROS_DEPENDS_LIB ${${DEPEND}_TARGETS})
  elseif(TARGET ${DEPEND}::${DEPEND})
    list(APPEND ROS_DEPENDS_LIB ${DEPEND}::${DEPEND})
  endif()
endforeach(DEPEND)

set(EXECUTABLES "")  # for installing later

message(STATUS "building ${PROJECT_NAME} node: ")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message(STATUS "  - ${NODE_NAME}")
  list(APPEND EXECUTABLES ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS})
  target_link_libraries(${NODE_NAME} ${ROS_DEPENDS_LIB} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")
endforeach(NODE_FILE)


message(STATUS "  Installing: ${EXECUTABLES}")
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

#############
## testing ##
#############

# Tests added here are run by `colcon test` and by the GitHub Actions build
# checks.  Example:
#
#   if(BUILD_TESTING)
#     find_package(ament_cmake_gtest REQUIRED)
#     ament_add_gtest(${PROJECT_NAME}_test test/my_test.cpp)
#   endif()
#
# and add <test_depend>ament_cmake_gtest</test_depend> to package.xml.

ament_package()
