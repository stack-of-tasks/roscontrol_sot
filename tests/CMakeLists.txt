# Copyright (C) 2017-2019 LAAS-CNRS
#
# Author: Olivier Stasse
#

add_executable(roscontrol_sot_hardware roscontrol_sot_hardware.cpp)
target_include_directories(
  roscontrol_sot_hardware
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(roscontrol_sot_hardware rcsot_controller
                      ${catkin_LIBRARIES} ${bullet_libraries})

# Add the library to wrap the test device.
set(DEVICE_NAME sot-test-device)
add_library(${DEVICE_NAME} SHARED ${DEVICE_NAME}.cpp)
target_link_libraries(${DEVICE_NAME} sot-core::sot-core)
install(TARGETS ${DEVICE_NAME} DESTINATION lib)

# build python submodule
string(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})

dynamic_graph_python_module(
  "sot/test/${PYTHON_LIBRARY_NAME}" ${DEVICE_NAME}
  sot-test-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
  "${CMAKE_CURRENT_SOURCE_DIR}/${DEVICE_NAME}-python.hh")
set_target_properties(
  sot-test-${PYTHON_LIBRARY_NAME}-wrap PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                  ../src) # FIXME: weird catkin
                                                          # / dg interraction

# Add the library to wrap the controller of Test.
set(CONTROLLER_NAME sot-test-controller)
add_library(${CONTROLLER_NAME} SHARED sot-test-controller.cpp)
target_include_directories(${CONTROLLER_NAME} SYSTEM
                           PUBLIC ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${CONTROLLER_NAME} ${DEVICE_NAME})
install(TARGETS ${CONTROLLER_NAME} DESTINATION lib)

if(${PYTHON_VERSION_MAJOR} GREATER 2)
  add_executable(sot-test-log sot-test-log.cc)
  target_link_libraries(sot-test-log rcsot_controller)

  add_python_unit_test("py-test-log" "tests/test_log.py" "python")
endif()
