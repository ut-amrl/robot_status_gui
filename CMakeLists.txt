PROJECT(robot_status_gui)
cmake_minimum_required(VERSION 2.6)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

set (CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -DNDEBUG")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package (Qt4 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include ( ${QT_USE_FILE} )

include_directories(${PROJECT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(src/)

set(libs roslib popt pthread rt)

qt4_wrap_cpp(GUIMONITOR_MOC_SRCS
  src/gui_topic.h
  src/gui_monitor.h
)


set (target gui_monitor)
ROSBUILD_ADD_EXECUTABLE(${target}
                        src/main.cpp
                        src/topic_config.cpp
                        src/gui_monitor.cpp
                        src/gui_topic.cpp
                        ${GUIMONITOR_MOC_SRCS})
add_dependencies(${target} ${GUIMONITOR_MOC_SRCS} rosbuild_precompile)
target_link_libraries(${target} ${libs} ${QT_LIBRARIES})

