# Install script for directory: /home/joshua/ROS_projects/pickAndPlaceKuka/src/kuka_experimental/kuka_kr5_support

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/joshua/ROS_projects/pickAndPlaceKuka/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/joshua/ROS_projects/pickAndPlaceKuka/build/kuka_experimental/kuka_kr5_support/catkin_generated/installspace/kuka_kr5_support.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr5_support/cmake" TYPE FILE FILES
    "/home/joshua/ROS_projects/pickAndPlaceKuka/build/kuka_experimental/kuka_kr5_support/catkin_generated/installspace/kuka_kr5_supportConfig.cmake"
    "/home/joshua/ROS_projects/pickAndPlaceKuka/build/kuka_experimental/kuka_kr5_support/catkin_generated/installspace/kuka_kr5_supportConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr5_support" TYPE FILE FILES "/home/joshua/ROS_projects/pickAndPlaceKuka/src/kuka_experimental/kuka_kr5_support/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr5_support" TYPE DIRECTORY FILES
    "/home/joshua/ROS_projects/pickAndPlaceKuka/src/kuka_experimental/kuka_kr5_support/config"
    "/home/joshua/ROS_projects/pickAndPlaceKuka/src/kuka_experimental/kuka_kr5_support/launch"
    "/home/joshua/ROS_projects/pickAndPlaceKuka/src/kuka_experimental/kuka_kr5_support/meshes"
    "/home/joshua/ROS_projects/pickAndPlaceKuka/src/kuka_experimental/kuka_kr5_support/urdf"
    )
endif()

