execute_process(COMMAND "/home/zhang/catkin_ws/build/multimaster_fkie/node_manager_fkie/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zhang/catkin_ws/build/multimaster_fkie/node_manager_fkie/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
