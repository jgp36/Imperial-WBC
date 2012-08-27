cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_BUILD_TYPE Debug)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE Debug)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_definitions (-Wall -pipe)
rosbuild_init ()

set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library (wbc_uta_opspace
  uta_opspace/strutil.cpp
  uta_opspace/ControllerNG.cpp
  uta_opspace/ControllerVel.cpp
  uta_opspace/ControllerIF.cpp
  uta_opspace/HelloGoodbyeSkill.cpp
  uta_opspace/DelayHistogram.cpp
  uta_opspace/TaskOriPostureSkill.cpp
  #uta_opspace/WriteSkill.cpp
  #uta_opspace/LetterManager.cpp
  uta_opspace/CartMultiPos.cpp
  uta_opspace/JointMultiPos.cpp
  uta_opspace/BaseMultiPos.cpp
  uta_opspace/RigidTf.cpp
  uta_opspace/SurfaceMotion.cpp
  uta_opspace/SurfaceOriMotion.cpp
  uta_opspace/AttachSurface.cpp
  uta_opspace/KnownAttachSurface.cpp
  )
