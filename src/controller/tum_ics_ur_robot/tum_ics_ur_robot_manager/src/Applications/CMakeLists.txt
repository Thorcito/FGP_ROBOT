cmake_minimum_required(VERSION 2.6)

project(UrRobotManagerApps CXX C)


# robot_script_manager_ur10
set(TARGET_NAME robot_script_manager_ur10)
set(QT_HEADERS
    ../ConsoleReader.h
)

set(SOURCES
    ${QT_HEADERS}
    ../ConsoleReader.cpp
)
QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

add_executable(${TARGET_NAME} main_${TARGET_NAME}.cpp ${SOURCES} ${HEADERS_MOC} ${QT_HEADERS})
add_dependencies(${TARGET_NAME} ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

target_link_libraries(${TARGET_NAME}
    tum_ics_ur_robot_manager
    ${QT_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
)

add_executable(testScriptLoader main_testScriptLoader.cpp)
add_dependencies(testScriptLoader ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

target_link_libraries(testScriptLoader
    tum_ics_ur_robot_manager
    ${QT_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
)

# robot_script_manager
#set(TARGET_NAME robot_script_manager)
#set(QT_HEADERS
#    ../ConsoleReader.h
#)

#set(SOURCES
#    ${QT_HEADERS}
#    ../ConsoleReader.cpp
#)
#QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

#add_executable(${TARGET_NAME} main_${TARGET_NAME}.cpp ${SOURCES} ${HEADERS_MOC} ${QT_HEADERS})
#add_dependencies(${TARGET_NAME} ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

#target_link_libraries(${TARGET_NAME}
#    tum_ics_ur_robot_manager
#    ${QT_LIBRARIES}
#)


# robot_arm_closed
#set(TARGET_NAME robot_arm_closed)
#set(QT_HEADERS
#    ../ConsoleReader.h
#)

#set(SOURCES
#    ${QT_HEADERS}
#    ../ConsoleReader.cpp
#)
#QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

#add_executable(${TARGET_NAME} main_${TARGET_NAME}.cpp ${SOURCES} ${HEADERS_MOC} ${QT_HEADERS})
#add_dependencies(${TARGET_NAME} ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

#target_link_libraries(${TARGET_NAME}
#    tum_ics_ur_robot_manager
#    ${catkin_LIBRARIES}
#    ${QT_LIBRARIES}
#)



# skill_list_test
#set(TARGET_NAME skill_list_test)
#set(QT_HEADERS
#    ../ConsoleReader.h
#)

#set(SOURCES
#    ${QT_HEADERS}
#    ../ConsoleReader.cpp
#)
#QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

#add_executable(${TARGET_NAME} main_${TARGET_NAME}.cpp ${SOURCES} ${HEADERS_MOC} ${QT_HEADERS})
#add_dependencies(${TARGET_NAME} ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

#target_link_libraries(${TARGET_NAME}
#    tum_ics_ur_robot_manager
#    ${catkin_LIBRARIES}
#    ${QT_LIBRARIES}
#)


# skill_list_test
#set(TARGET_NAME skill_manager_test)
#set(QT_HEADERS
#    ../ConsoleReader.h
#)

#set(SOURCES
#    ${QT_HEADERS}
#    ../ConsoleReader.cpp
#)
#QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

#add_executable(${TARGET_NAME} main_${TARGET_NAME}.cpp ${SOURCES} ${HEADERS_MOC} ${QT_HEADERS})
#add_dependencies(${TARGET_NAME} ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS})

#target_link_libraries(${TARGET_NAME}
#    tum_ics_ur_robot_manager
#    ${catkin_LIBRARIES}
#    ${QT_LIBRARIES}
#)


# binary traget install
set(APP_TARGETS
#    robot_script_manager
#    skill_list_test
    robot_script_manager_ur10
)

install(TARGETS ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



