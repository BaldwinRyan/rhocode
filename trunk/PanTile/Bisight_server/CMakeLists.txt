cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/home/noam/ros_workspace/PanTilt/Biclops/TracLabs/Motion/libPMD/include -I/home/noam/ros_workspace/PanTilt/Biclops/TracLabs/Motion/libUtils/include")

include("${CMAKE_SOURCE_DIR}/FindBiclops.cmake")
include_directories(${biclops_INCLUDE_DIRS} ${pmd_INCLUDE_DIRS} ${util_INCLUDE_DIRS})
link_directories(${biclops_LIBRARY_DIRS} ${pmd_LIBRARY_DIRS} ${util_LIBRARY_DIRS})
#rosbuild_add_library(${PROJECT_NAME} src/comport.c)
#rosbuild_add_library(${PROJECT_NAME} src/bisight_driver.cpp)
rosbuild_add_executable(BiclopsServer src/BiclopsAccess.cpp src/BiclopsServer.cpp)
#rosbuild_add_executable(BisightNode src/bisight_node.cpp)

#rosbuild_add_executable(Bisight_test src/bisight_test.cpp)

include_directories( /home/noam/ros_workspace/PanTilt/Biclops/TracLabs/Motion/ )
#target_link_libraries(BiclopsAccess /home/noam/ros_workspace/PanTilt/Biclops/TracLabs/Motion/libPMD /home/noam/ros_workspace/PanTilt/Biclops/TracLabs/Motion/libUtils)
target_link_libraries(BiclopsServer Biclops Utils PMD)
#target_link_libraries(Bisight_test)
