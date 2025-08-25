cmake_minimum_required(VERSION 2.8.3)
project(tum_ics_ur_robot_manager)

# The TUM ICS CMake Tools
#include(/usr/share/cmake/CMakeTools/ics_tools.cmake)

#Set the RPATH in the binaries with the path information of the library dependencies
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    tum_ics_ur_robot_lli
    tum_ics_ur_robot_controllers
    tum_ics_ur_robot_msgs
    tum_ics_msgs
    tum_ics_lacquey_gripper_msgs
)

set(DEPENDS_VAR system_lib libqt4-dev)
if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
    set(DEPENDS_VAR )
endif()

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        tum_ics_ur_robot_msgs
        tum_ics_ur_robot_lli
        tum_ics_msgs tum_ics_ur_robot_controllers
        tum_ics_lacquey_gripper_msgs
    LIBRARIES tum_ics_ur_robot_manager
    DEPENDS ${DEPENDS_VAR}
)


##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

##############################################################################
# TUM-ICS Math Lib
##############################################################################
find_package(TMath REQUIRED)
add_definitions(${TMATH_DEFINITIONS})
include_directories(${TMATH_INCLUDE_DIRS})

##############################################################################
# TUM-ICS RT Threads
##############################################################################
find_package(RtThreads REQUIRED)
include_directories(${RTTHREADS_INCLUDE_DIR})
add_definitions(${RTTHREADS_DEFINITIONS})

##############################################################################
# Qt Library
##############################################################################

message("${BGREEN}+++${NOCOLOR} Compile with ${BYELLOW}Qt5${NOCOLOR}")
find_package(Qt5 COMPONENTS Core Gui Network REQUIRED)


# find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)

# # add Qt Modules
# set(QT_USE_QTNETWORK TRUE)
# include(${QT_USE_FILE})


# DON'T USE (enforces Q_SIGNAL etc. macros)
add_definitions(-DQT_NO_KEYWORDS)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)


# needed for Qt generated files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)



###############################################################################
#   Build libraries
###############################################################################

file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
	include/*.h)

set(SOURCES
    ${QT_HEADERS}
    src/ScriptLoader.cpp
    src/ScriptManager.cpp
    src/RobotScriptManager.cpp
    src/Skill.cpp
    src/SkillTransition.cpp
    src/SkillList.cpp
    src/SkillTransitionManager.cpp
    src/SkillManager.cpp
    src/SkillServer.cpp
    src/RobotArmClosed.cpp
)


# feed moc
QT5_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})




##############################################################################
# Libraries
##############################################################################
# add executables
add_library(tum_ics_ur_robot_manager ${SOURCES} ${HEADERS} ${HEADERS_MOC} ${FORMS_UI} ${RESOURCES_RC})
add_dependencies(tum_ics_ur_robot_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# link target against libs
target_link_libraries(tum_ics_ur_robot_manager
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tum_ics_ur_robot_lli_LIBRARIES}
    ${QT_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
)


target_link_libraries(tum_ics_ur_robot_manager
    Qt5::Core
    Qt5::Gui
    Qt5::Network
)


##############################################################################
# Applications
##############################################################################
set(QT_HEADERS
    include/${PROJECT_NAME}/ConsoleReader.h
)
# QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

# feed moc
QT5_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})



set(APP_TARGETS
    robot_script_manager_ur10
    robot_script_manager
    skill_list_test
    skill_manager_test
    robot_arm_closed
    testScriptLoader

)

foreach(target ${APP_TARGETS})
        add_executable(${target} src/Applications/main_${target}.cpp src/ConsoleReader.cpp ${HEADERS_MOC} ${QT_HEADERS})
        add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

        target_link_libraries(${target}
            ${catkin_LIBRARIES}
            ${roscpp_LIBRARIES}
            ${tum_ics_ur_robot_lli_LIBRARIES}
            ${QT_LIBRARIES}
            ${RTTHREADS_LIBRARIES}
            ${TMATH_LIBRARIES}
            tum_ics_ur_robot_manager
        )

        target_link_libraries(${target}
            Qt5::Core
            Qt5::Gui
            Qt5::Network
        )

endforeach()


##############################################################################
# Install
##############################################################################

###############################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the
#   debtools. This function is controlled with the global variable -DDEB_BUILD
#   (true/false)
#ics_ros_debgen()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(INS_TARGETS
    tum_ics_ur_robot_manager
    ${APP_TARGETS}
)

install(TARGETS ${INS_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch srv)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
