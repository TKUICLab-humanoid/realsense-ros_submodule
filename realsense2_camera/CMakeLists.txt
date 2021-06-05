cmake_minimum_required(VERSION 2.8.3)
project(realsense2_camera)
add_compile_options(-std=c++11)

option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(catkin REQUIRED COMPONENTS
    message_filters
    pluginlib
	tf2_ros
	tf2_geometry_msgs
	tf2
	geometry_msgs
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    nodelet
    cv_bridge
    image_transport
    tf
    ddynamic_reconfigure
    diagnostic_updater
    tku_libs
    tku_msgs
    )

if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()

if(SET_USER_BREAK_AT_STARTUP)
	message("GOT FLAG IN CmakeLists.txt")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBPDEBUG")
endif()

if (WIN32)
find_package(realsense2 CONFIG REQUIRED)
else()
find_package(realsense2 2.45.0)
endif()

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

if (WIN32)
else()
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
endif()

add_message_files(
    FILES
    IMUInfo.msg
    Extrinsics.msg
    IMUdata.msg
    )

generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
    )

set(CMAKE_NO_SYSTEM_FROM_IMPORTED true)
include_directories(
    include
    ${realsense2_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
    )

find_package(Boost REQUIRED COMPONENTS system thread)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(cfg/ImuFilterMadgwick.cfg)
find_package(OpenCV REQUIRED)

# RealSense ROS Node
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
    nodelet
    cv_bridge
    image_transport
    ddynamic_reconfigure
    nav_msgs
    DEPENDS Boost
	INCLUDE_DIRS
	LIBRARIES imu_filter imu_filter_nodelet
    )
# create imu_filter library
add_library (imu_filter src/imu_filter/imu_filter.cpp  src/imu_filter/imu_filter_ros.cpp src/imu_filter/stateless_orientation.cpp)
add_dependencies(imu_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_nodelet library
add_library (imu_filter_nodelet src/imu_filter/imu_filter_nodelet.cpp)
add_dependencies(imu_filter_nodelet ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_nodelet imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_node executable
add_executable(imu_filter_node src/imu_filter/imu_filter_node.cpp)
add_dependencies(imu_filter_node ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_node imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS imu_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS imu_filter imu_filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES imu_filter_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
add_library(${PROJECT_NAME}
    include/constants.h
    include/realsense_node_factory.h
    include/base_realsense_node.h
    include/t265_realsense_node.h
    src/realsense_node_factory.cpp
    src/base_realsense_node.cpp
    src/t265_realsense_node.cpp
    )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}
  PRIVATE ${realsense2_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )
add_executable(realsense_cam 
#    src/main.cpp 
    src/Camera_Controls.cpp
    src/RGB_camera_controls.cpp
#    src/Get_D435i_IMU.cpp
#    include/main.h 
    include/Camera_Controls/Camera_Controls.h
    include/RGB_camera_controls/RGB_camera_controls.h
#    include/Get_D435i_IMU/Get_D435i_IMU.h
    )

#add_executable(realsense_IMU 
#   src/Get_D435i_IMU.cpp
#   include/Get_D435i_IMU/Get_D435i_IMU.h
#    )
target_link_libraries(realsense_cam
#  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${realsense2_LIBRARY}
)
#target_link_libraries(realsense_IMU
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${realsense2_LIBRARY}
#)
if(WIN32)
set_target_properties(${realsense2_LIBRARY} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
target_link_libraries(${PROJECT_NAME}
    realsense2::realsense2 
    realsense2::realsense-file
    )
endif()


# Install nodelet library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Install header files
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

# Install rviz files
install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )

# Install xml files
install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)