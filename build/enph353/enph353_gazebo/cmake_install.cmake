# Install script for directory: /home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo/srv" TYPE FILE FILES
    "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv"
    "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo/cmake" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazebo-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/devel/include/enph353_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/devel/share/roseus/ros/enph353_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/devel/share/common-lisp/ros/enph353_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/devel/share/gennodejs/ros/enph353_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/devel/lib/python2.7/dist-packages/enph353_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/devel/lib/python2.7/dist-packages/enph353_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo/cmake" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazebo-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo/cmake" TYPE FILE FILES
    "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazeboConfig.cmake"
    "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo/cmake" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazebo-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo/cmake" TYPE FILE FILES
    "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazeboConfig.cmake"
    "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/build/enph353/enph353_gazebo/catkin_generated/installspace/enph353_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/enph353_gazebo" TYPE FILE FILES "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/package.xml")
endif()

