# Copyright (C) 2017-2020 LAAS-CNRS
#
# Author: Olivier Stasse Author: Guilhem Saurel
#
cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_DESCRIPTION roscontrol_sot)
set(PROJECT_NAME roscontrol_sot)
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options

# Project configuration
set(CXX_DISABLE_WERROR True)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)
include(cmake/ros.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 ENFORCE)

# Project dependencies
findpython()
search_for_boost_python()
add_required_dependency(bullet)
add_required_dependency(urdfdom)

find_package(
  catkin REQUIRED
  COMPONENTS pal_hardware_interfaces
             controller_interface
             controller_manager
             pal_common_msgs
             roscpp
             std_msgs
             control_msgs
             sensor_msgs
             realtime_tools
             rospy)
find_package(temperature_sensor_controller)

add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
add_project_dependency(dynamic_graph_bridge REQUIRED)
add_project_dependency(sot-core REQUIRED)
add_project_dependency(pinocchio REQUIRED)
add_project_dependency(control_toolbox REQUIRED)

# Detect the controller interface version to switch code
if(controller_interface_FOUND)
  if(${controller_interface_VERSION} VERSION_GREATER "0.2.5")
    add_definitions(-DCONTROLLER_INTERFACE_KINETIC)
    if(${controller_interface_VERSION} VERSION_GREATER "0.19.0")
      add_definitions(-DCONTROLLER_INTERFACE_NOETIC)
    endif()
  endif()
endif()

# Detect if temperature sensor controller package is found if yes then it is a
# PAL Robotics Forked code.

add_library(rcsot_controller src/roscontrol-sot-controller.cpp src/log.cpp)
target_include_directories(
  rcsot_controller SYSTEM PUBLIC ${catkin_INCLUDE_DIRS}
                                 $<INSTALL_INTERFACE:include>)
target_link_libraries(
  rcsot_controller ${catkin_LIBRARIES} ${bullet_LIBRARIES} sot-core::sot-core
  dynamic_graph_bridge::sot_loader ${control_toolbox_LIBRARIES})
if(temperature_sensor_controller_FOUND)
  target_compile_definitions(rcsot_controller
                             PUBLIC TEMPERATURE_SENSOR_CONTROLLER)
  message(
    "temperature_sensor_controller_LIBRARIES: ${temperature_sensor_controller_LIBRARIES}"
  )
  target_include_directories(
    rcsot_controller SYSTEM
    PUBLIC ${temperature_sensor_controller_INCLUDE_DIRS})
  target_link_libraries(rcsot_controller
                        ${temperature_sensor_controller_LIBRARIES})
endif()

if(SUFFIX_SO_VERSION)
  set_target_properties(rcsot_controller PROPERTIES SOVERSION
                                                    ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS rcsot_controller
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_executable(roscontrol-sot-parse-log src/roscontrol-sot-parse-log.cc)
install(
  TARGETS roscontrol-sot-parse-log
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION bin)

foreach(dir config launch)
  install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
endforeach(dir)

add_subdirectory(tests)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
