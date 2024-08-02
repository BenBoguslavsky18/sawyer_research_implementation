execute_process(COMMAND "/home/airlab5/ben_ws/build/intera_interface/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/airlab5/ben_ws/build/intera_interface/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
