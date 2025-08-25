cmake_minimum_required(VERSION 2.8.0)
project(tum_ics_ur_robot_msgs)

#Set the RPATH in the binaries with the path information of the library dependencies
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
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
      Special_cmd.msg
      PlotRobotData.msg
      PlotRobotFullData.msg
      RobotTF.msg
      JointPoseInt.msg
      JointPoseFloat.msg
      ControlData.msg
      RobotData.msg
      FloatValue.msg
      ControlEventEval.msg
      ForceQPData.msg
      ControlSkinDstbTorqueData.msg
)

# Generate services
add_service_files(
  DIRECTORY srv
  FILES
    setScriptManagerState.srv
    getScriptManagerStates.srv
    UpdateKDPsych.srv
    GetContactFlag.srv
    setControllerCmd.srv
    GetXefW.srv
    set_operation_mode.srv
    Move2Object.srv
    SetTemperatureClass.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#    DIRECTORY
#        action
#    FILES
#        Playskill.action
#   Action1.action
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES  
    actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)


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
 CATKIN_DEPENDS actionlib actionlib_msgs std_msgs message_runtime geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

###############################################################################
# Special c++ compiler flags
###############################################################################

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp -O3" )


##############################################################################
# Install
##############################################################################

###############################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the
#   debtools. This function is controlled with the global variable -DDEB_BUILD
#   (true/false)
#include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
#ics_ros_debgen()

foreach(dir msg srv action)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



