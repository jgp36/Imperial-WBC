# Install script for directory: /home/mimkuka/ros_workspace/whole_body_control/wbc_display

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/wbc_display:/opt/ros/fuerte/lib:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/ocl/lib:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/log4cpp/../install/lib:/home/mimkuka/ros_workspace/whole_body_control/wbc_core/lib:/home/mimkuka/ros_workspace/orocos_core/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/install/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display" TYPE SHARED_LIBRARY FILES "/home/mimkuka/ros_workspace/whole_body_control/wbc_display/lib/orocos/gnulinux/libwbc_display-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so"
         OLD_RPATH "/opt/ros/fuerte/lib:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/ocl/lib:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/log4cpp/../install/lib:/home/mimkuka/ros_workspace/whole_body_control/wbc_core/lib:/home/mimkuka/ros_workspace/orocos_core/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/wbc_display:/opt/ros/fuerte/lib:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/ocl/lib:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/log4cpp/../install/lib:/home/mimkuka/ros_workspace/whole_body_control/wbc_core/lib:/home/mimkuka/ros_workspace/orocos_core/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/mimkuka/ros_workspace/orocos_core/orocos_toolchain/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/wbc_display/libwbc_display-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/wbc_display" TYPE FILE FILES "/home/mimkuka/ros_workspace/whole_body_control/wbc_display/src/wbc_display-component.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mimkuka/ros_workspace/whole_body_control/wbc_display/wbc_display-gnulinux.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/mimkuka/ros_workspace/whole_body_control/wbc_display/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/mimkuka/ros_workspace/whole_body_control/wbc_display/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
