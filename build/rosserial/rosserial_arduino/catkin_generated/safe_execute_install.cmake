execute_process(COMMAND "/home/richard/Rover2018/build/rosserial/rosserial_arduino/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/richard/Rover2018/build/rosserial/rosserial_arduino/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
