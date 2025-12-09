execute_process(COMMAND "/home/hzy/catkin_ws/build/skid4wd_ws/src/misc_func/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hzy/catkin_ws/build/skid4wd_ws/src/misc_func/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
