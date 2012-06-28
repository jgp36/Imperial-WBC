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

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library( wbc_stb
  src/udp_osi.cxx)

rosbuild_add_executable(servo src/servo.cpp)
target_link_libraries(servo wbc_stb)

rosbuild_add_executable(robottest src/robottest.cpp)
target_link_libraries(robottest wbc_stb)

rosbuild_add_executable(fakerobot src/fakerobot.cpp)
target_link_libraries(fakerobot wbc_stb)

rosbuild_add_executable(fakecamera src/fakecamera.cpp)
target_link_libraries(fakecamera wbc_stb)