cmake_minimum_required(VERSION 2.8.3)
project(tum_ics_ur_robot_controllers)

#Set the RPATH in the binaries with the path information of the library dependencies
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tum_ics_ur_robot_msgs
  tum_ics_ur_robot_lli
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)


##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp" )

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
  LIBRARIES tum_ics_ur_robot_controllers
  CATKIN_DEPENDS tum_ics_ur_robot_msgs tum_ics_ur_robot_lli roscpp sensor_msgs std_msgs tf tf_conversions
  DEPENDS ${DEPENDS_VAR}
)


find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)
message("${BGREEN}+++${NOCOLOR} Compile with ${BYELLOW}Qt5${NOCOLOR}")


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR} # needed for Qt generated files
  ${CMAKE_CURRENT_SOURCE_DIR} # needed for Qt generated files
)
    
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp -O3" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3" )
set(CMAKE_BUILD_TYPE Release)

###########
## Build ##
###########

file(GLOB_RECURSE UR_CTRL_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
	include/*.h)

##############################################################################
# Sections Library. Here goes the src code of your controller
##############################################################################

add_library(tum_ics_ur_robot_controllers
    src/DirectJointControl.cpp
    src/SimpleEffortControl.cpp
    src/SOSMControl.cpp
    src/PIDControl.cpp
    src/SOSMOpControl.cpp
    src/JointImpedanceControl.cpp
    src/XImpedanceControl.cpp
    src/TestEffortControl.cpp
    ##New Controls
    src/SkillControls/GControl.cpp
    src/SkillControls/SplineJointControl.cpp
    src/SkillControls/JointControl.cpp
    src/SkillControls/SplineCartesianControl.cpp
    src/SkillControls/CartesianControl.cpp
    src/SkillControls/DControl.cpp
   ${UR_CTRL_INC}
 )
target_link_libraries(tum_ics_ur_robot_controllers
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
    ${tum_ics_ur_robot_lli_LIBRARIES}
)

target_link_libraries(tum_ics_ur_robot_controllers Qt5::Core)
# target_link_libraries(tum_ics_ur_robot_controllers Qt5::Core Qt5::Network Qt5::Widgets)


add_dependencies(tum_ics_ur_robot_controllers
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

##############################################################################
# Applications. Here goes your applications (using the controller class you defined in the library above)
##############################################################################
set(APP_TARGETS
    testJointControl
    testJointCtrlPublisher
    testMultipleControls
    testRobotTime
)

foreach(target ${APP_TARGETS})
        add_executable(${target} src/Applications/${target}.cpp)
        add_dependencies(${target}
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS}
        )
        target_link_libraries(${target}
            ${QT_LIBRARIES}
            ${catkin_LIBRARIES}
            ${roscpp_LIBRARIES}
            ${RTTHREADS_LIBRARIES}
            ${TMATH_LIBRARIES}
            ${tum_ics_ur_robot_lli_LIBRARIES}
            tum_ics_ur_robot_controllers
        )
        
        target_link_libraries(${target} Qt5::Core Qt5::Network Qt5::Widgets)

endforeach()

##############################################################################
# Install
##############################################################################

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the debtools
# This function is controlled with the global variable -DDEB_BUILD (true/false)
#include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
#ics_ros_debgen()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(INS_TARGETS
    tum_ics_ur_robot_controllers
    ${APP_TARGETS}
)

install(TARGETS ${INS_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


