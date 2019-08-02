# Copyright (C) 2017-2019 LAAS-CNRS
#
# Author: Olivier Stasse
#

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIRECTORY}/tests)

ADD_EXECUTABLE(roscontrol_sot_hardware roscontrol_sot_hardware.cpp)
pkg_config_use_dependency(roscontrol_sot_hardware controller_manager
  NO_INCLUDE_SYSTEM)
target_link_libraries(roscontrol_sot_hardware
  ${catkin_LIBRARIES}
  ${bullet_libraries}
  )
