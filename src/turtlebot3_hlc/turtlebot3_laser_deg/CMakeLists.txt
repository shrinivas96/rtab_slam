cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_laser_deg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
# add the packages that you want to use throughout
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  occupancy_grid_utils
)

###################################
## catkin specific configuration ##
###################################

# add packages you want to use
# the assumption is that by adding 'support_utils' now the system knows which libraries need to be ueed internally?
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    support_utils
    # ${PROJECT_NAME} # this was written previously but without an understanding of why. library name was changed since then.
  CATKIN_DEPENDS 
    roscpp 
    rospy
    sensor_msgs
    std_msgs
    nav_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# we make our own library with a class so we need to add them here
# and they need to be linked. this is not the same as doing for the nodes
# the use of support_utils needs to be consistent b/w add_lib and target_link_lib
add_library(support_utils 
  src/returnMap.cpp
  src/readFromFile.cpp
  src/simulateScan
)
target_link_libraries(support_utils ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node 
  src/turtlebot3_laser_deg_node.cpp
  src/TurtlebotHighlevelController.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node support_utils ${catkin_LIBRARIES})

# this is a self added node
add_executable(map_ray_trace_node
  src/map_ray_trace_node.cpp
  src/rayTracer.cpp
)

# and here is where you need your new library
target_link_libraries(map_ray_trace_node support_utils ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node support_utils map_ray_trace_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h**"
)