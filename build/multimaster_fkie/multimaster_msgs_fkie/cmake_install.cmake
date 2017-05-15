# Install script for directory: /home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/zhang/catkin_ws/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multimaster_msgs_fkie/msg" TYPE FILE FILES
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/Capability.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/LinkState.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/LinkStatesStamped.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/MasterState.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/ROSMaster.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/SyncMasterInfo.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/SyncServiceInfo.msg"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/msg/SyncTopicInfo.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multimaster_msgs_fkie/srv" TYPE FILE FILES
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/srv/DiscoverMasters.srv"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/srv/GetSyncInfo.srv"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/srv/ListDescription.srv"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/srv/ListNodes.srv"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/srv/LoadLaunch.srv"
    "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/srv/Task.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multimaster_msgs_fkie/cmake" TYPE FILE FILES "/home/zhang/catkin_ws/build/multimaster_fkie/multimaster_msgs_fkie/catkin_generated/installspace/multimaster_msgs_fkie-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhang/catkin_ws/devel/include/multimaster_msgs_fkie")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhang/catkin_ws/devel/share/common-lisp/ros/multimaster_msgs_fkie")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/zhang/catkin_ws/devel/lib/python2.7/dist-packages/multimaster_msgs_fkie")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zhang/catkin_ws/devel/lib/python2.7/dist-packages/multimaster_msgs_fkie")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhang/catkin_ws/build/multimaster_fkie/multimaster_msgs_fkie/catkin_generated/installspace/multimaster_msgs_fkie.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multimaster_msgs_fkie/cmake" TYPE FILE FILES "/home/zhang/catkin_ws/build/multimaster_fkie/multimaster_msgs_fkie/catkin_generated/installspace/multimaster_msgs_fkie-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multimaster_msgs_fkie/cmake" TYPE FILE FILES
    "/home/zhang/catkin_ws/build/multimaster_fkie/multimaster_msgs_fkie/catkin_generated/installspace/multimaster_msgs_fkieConfig.cmake"
    "/home/zhang/catkin_ws/build/multimaster_fkie/multimaster_msgs_fkie/catkin_generated/installspace/multimaster_msgs_fkieConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multimaster_msgs_fkie" TYPE FILE FILES "/home/zhang/catkin_ws/src/multimaster_fkie/multimaster_msgs_fkie/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

