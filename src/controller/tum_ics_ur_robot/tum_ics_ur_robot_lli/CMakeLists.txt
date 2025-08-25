cmake_minimum_required(VERSION 2.8.3)
project(tum_ics_ur_robot_lli)

#Set the RPATH in the binaries with the path information of the library dependencies
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tum_ics_ur_robot_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
)

##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp -fno-permissive" )

##############################################################################
# TUM-ICS Math Lib
##############################################################################
find_package(TMath REQUIRED)
add_definitions(${TMATH_DEFINITIONS})
include_directories(${TMATH_INCLUDE_DIRS})

##############################################################################
# RT Threads
##############################################################################
find_package(RtThreads REQUIRED)
include_directories(${RTTHREADS_INCLUDE_DIR})
add_definitions(${RTTHREADS_DEFINITIONS})

##############################################################################
# Qt Library
##############################################################################
# find_package(Qt4 REQUIRED)
#SET(QT_USE_QTOPENGL TRUE)
# SET(QT_USE_QTNETWORK TRUE)


# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp -fno-permissive" )
# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror=return-type" )

#message("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# include(${QT_USE_FILE})
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
message("${BGREEN}+++${NOCOLOR} Compile with ${BYELLOW}Qt5${NOCOLOR}")
find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)

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
  LIBRARIES tum_ics_ur_robot_lli tum_ics_ur_robot_interface_lli tum_ics_ur_robot_control_lli
  CATKIN_DEPENDS tum_ics_ur_robot_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions visualization_msgs
  DEPENDS ${DEPENDS_VAR}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# needed for Qt generated files (uic)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

###########
## Build ##
###########

file(GLOB_RECURSE UR_ROBOT_LLI_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
	include/*.h)


##############################################################################
# Sections Library Enable for Development
##############################################################################
add_library(tum_ics_ur_robot_interface_lli
   src/JointState.cpp
   src/RobotTime.cpp
   src/RobotInterface/CommInterface.cpp
   src/RobotInterface/ScriptLoader.cpp
   src/RobotInterface/TcpSocketTools.cpp
   src/RobotInterface/DataUnpacker.cpp
   src/RobotInterface/RobotStateClient.cpp
   src/RobotInterface/RobotStatePub.cpp
 )
target_link_libraries(tum_ics_ur_robot_interface_lli
    ${roscpp_LIBRARIES}
    ${catkin_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
    ${QT_LIBRARIES}
)


target_link_libraries(tum_ics_ur_robot_interface_lli Qt5::Core Qt5::Network Qt5::Widgets)


add_dependencies(tum_ics_ur_robot_interface_lli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(tum_ics_ur_robot_control_lli
   src/RobotControllers/Controller.cpp
   src/RobotControllers/ControllerList.cpp
   src/RobotControllers/ControlEffort.cpp
   src/RobotControllers/ControlPosition.cpp
   src/RobotControllers/ControlVelocity.cpp
   src/RobotControllers/ControlEffort.cpp
 )
target_link_libraries(tum_ics_ur_robot_control_lli
    tum_ics_ur_robot_interface_lli
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${roscpp_LIBRARIES}
)


target_link_libraries(tum_ics_ur_robot_control_lli Qt5::Core)


add_dependencies(tum_ics_ur_robot_control_lli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(tum_ics_ur_robot_lli
   src/Robot/RobotArm.cpp
   src/Robot/RobotArmConstrained.cpp
   src/Robot/RobotArmExt.cpp
   src/Robot/KinematicModel.cpp
   src/Robot/DynamicModel.cpp
   ${UR_ROBOT_LLI_INC} #To see the include folder in qtcreator indigo <
 )
target_link_libraries(tum_ics_ur_robot_lli
    tum_ics_ur_robot_control_lli
    tum_ics_ur_robot_interface_lli
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
)

target_link_libraries(tum_ics_ur_robot_lli Qt5::Core)

add_dependencies(tum_ics_ur_robot_lli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


##############################################################################
# Applications
##############################################################################

set(QT_HEADERS
    src/ConsoleReader.h
)

set(SOURCES
    ${QT_HEADERS}
    src/ConsoleReader.cpp
)

# feed moc
# QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})
QT5_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})


set(APP_TARGETS
    robot_state_client
    comm_intf_test
)

foreach(target ${APP_TARGETS})
        add_executable(${target} src/Applications/main_${target}.cpp ${SOURCES} ${HEADERS_MOC})
        add_dependencies(${target} ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

        target_link_libraries(${target}
            tum_ics_ur_robot_lli
            ${catkin_LIBRARIES}
            ${roscpp_LIBRARIES}
            ${QT_LIBRARIES}
        )

        target_link_libraries(${target} Qt5::Core)


endforeach()

#############
## Install ##
#############

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the debtools
# This function is controlled with the global variable -DDEB_BUILD (true/false)
#include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
#ics_ros_debgen()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(INS_TARGETS
    tum_ics_ur_robot_lli
    tum_ics_ur_robot_interface_lli
    tum_ics_ur_robot_control_lli
    ${APP_TARGETS}
)

install(TARGETS ${INS_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#foreach(dir launch meshes ...)
#    install(DIRECTORY ${dir}/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)




