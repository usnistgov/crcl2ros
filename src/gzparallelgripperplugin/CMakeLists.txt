cmake_minimum_required(VERSION 2.8.3)
project(gzparallelgripperplugin)

# copy library to gazebo plugin area - use env var for destination folder
SET(CMAKE_INSTALL_PREFIX $ENV{GZPLUGINDIR} )
message ( "gzparallelgripperplugin install at ${CMAKE_INSTALL_PREFIX}" )
#


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    gazebo_plugins
    roscpp
    std_msgs
    std_srvs
    tf
    geometry_msgs
    message_generation)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fix_tray_objects
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_service_files(FILES
  GripCommand.srv
)

include_directories(
  include ${catkin_INCLUDE_DIRS}${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-format-security ${CMAKE_CXX_FLAGS}")

add_library(gzparallelgripperplugin
    src/gzparallelgripperplugin.cpp
)
add_dependencies(gzparallelgripperplugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gzparallelgripperplugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)


install(TARGETS gzparallelgripperplugin
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                 
