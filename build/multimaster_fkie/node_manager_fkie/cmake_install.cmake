# Install script for directory: /home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie

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
  INCLUDE("/home/zhang/catkin_ws/build/multimaster_fkie/node_manager_fkie/catkin_generated/safe_execute_install.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhang/catkin_ws/build/multimaster_fkie/node_manager_fkie/catkin_generated/installspace/node_manager_fkie.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/node_manager_fkie/cmake" TYPE FILE FILES
    "/home/zhang/catkin_ws/build/multimaster_fkie/node_manager_fkie/catkin_generated/installspace/node_manager_fkieConfig.cmake"
    "/home/zhang/catkin_ws/build/multimaster_fkie/node_manager_fkie/catkin_generated/installspace/node_manager_fkieConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/node_manager_fkie" TYPE FILE FILES "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/node_manager_fkie" TYPE PROGRAM FILES
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/nodes/node_manager"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/nodes/dynamic_reconfigure"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/scripts/remote_nm.py"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/scripts/respawn"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/scripts/reduced_nm.py"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/node_manager_fkie" TYPE DIRECTORY FILES
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/images"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/rqt_icons"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/node_manager_fkie" TYPE FILE FILES
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./README.rst"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./plugin.xml"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/node_manager_fkie" TYPE FILE FILES
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/MasterTab.ui"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/LaunchFilesDockWidget.ui"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/LogDockWidget.ui"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/PasswordInput.ui"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/MainWindow.ui"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/SettingsDockWidget.ui"
    "/home/zhang/catkin_ws/src/multimaster_fkie/node_manager_fkie/./src/node_manager_fkie/GUI.qrc"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

