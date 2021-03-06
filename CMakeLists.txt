cmake_minimum_required(VERSION 2.8.3)
project(pal_camera_client)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs  
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pal_camera_client
  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(pal_camera_client
            src/cameraClient.cpp src/stereoCameraClient.cpp src/utils.cpp
)
target_link_libraries(pal_camera_client ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS pal_camera_client
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

