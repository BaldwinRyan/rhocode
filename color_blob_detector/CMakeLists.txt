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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(hue_detector_monitor src/hue_detector_3d_monitor.cpp)
#rosbuild_add_executable(sat_detector_monitor src/sat_detector_3d_monitor.cpp)
#rosbuild_add_executable(multiple_hue_detector_monitor src/multiple_hue_detector_3d_monitor.cpp)
#rosbuild_add_executable(hue_detector src/color_blob_3d_test.cpp)
#rosbuild_add_executable(hue_detector_server src/hue_detector_3d_server.cpp)
#rosbuild_add_executable(hue_detector_client src/hue_detector_3d_client.cpp)
rosbuild_add_executable(hue src/color_blob_2d_test.cpp)
rosbuild_add_executable(hue_old src/color_blob_2d_test_nathan.cpp)
rosbuild_add_executable(object_modeling_distribution src/object_modeling_distribution.cpp)
rosbuild_add_executable(new_distribution src/new_color_blob_detector.cpp)
#rosbuild_add_executable(triangulation src/localization.cpp)
target_link_libraries(hue feature_tracker)
#target_link_libraries(hue_detector_monitor feature_tracker)
#target_link_libraries(hue_detector feature_tracker)
#target_link_libraries(hue_detector_server feature_tracker)


