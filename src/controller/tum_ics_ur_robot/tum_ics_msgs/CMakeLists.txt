##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(tum_ics_msgs)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp" )

# Set the rpath (to build deb package)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

##############################################################################
# Messages, Services and Actions
##############################################################################
add_message_files(
   DIRECTORY msg
   FILES 
  ControlData.msg
  Context.msg
  ObjectPose.msg
  VisualInfo.msg
  ActiveSkill.msg
  ForceQPData.msg
  JointPositions.msg
  JointTorques.msg
)

# Generate services
add_service_files(
  DIRECTORY srv
  FILES
  SendNewAction.srv
  PlaySkill.srv
  StopSkill.srv
  GetSkillState.srv
  GetActiveSkill.srv
)

## Generate actions in the 'action' folder
add_action_files(
    DIRECTORY
        action
    FILES
        Playskill.action

)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES  
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

#include_directories(include)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib actionlib_msgs std_msgs message_runtime geometry_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

#############
## Install ##
#############

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the debtools
# This function is controlled with the global variable -DDEB_BUILD (true/false)
#include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
#ics_ros_debgen()

foreach(dir msg srv action)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

