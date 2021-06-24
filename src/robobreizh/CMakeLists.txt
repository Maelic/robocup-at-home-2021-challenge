cmake_minimum_required(VERSION 2.8.3)
project(robobreizh)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    cmake_modules
    eigen_conversions
    tf
    )

# PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
  message(STATUS "Library GPD found in ${GPD_LIB}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()
include_directories(${GPD_LIB_INCLUDE_DIR})
message(STATUS "gpd_include_dir: ${GPD_LIB_INCLUDE_DIR}")

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")

add_message_files(FILES CloudIndexed.msg CloudSources.msg GraspConfig.msg GraspConfigList.msg DetectedObj.msg BoundingBoxCoord.msg GraspServerRequest.msg)

add_service_files(FILES detect_grasps.srv object_detection.srv grasping.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs )

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs
DEPENDS PCL
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_detect_grasps_server src/robobreizh/grasp_detection_server.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_server2 src/robobreizh/grasp_detection_server2.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_detect_grasps_server PROPERTIES OUTPUT_NAME detect_grasps_server PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_server2 PROPERTIES OUTPUT_NAME detect_grasps_server2 PREFIX "")

## Declare a C++ library
add_library(${PROJECT_NAME}_grasp_plotter src/${PROJECT_NAME}/grasp_plotter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detect_grasps_server
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_detect_grasps_server2
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})
## Python installs
install(PROGRAMS
  scripts/utils.py
  scripts/move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
