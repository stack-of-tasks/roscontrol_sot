# Copyright (C) 2017-2019 LAAS-CNRS
#
# Author: Olivier Stasse
#

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
set(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

ADD_EXECUTABLE(roscontrol_sot_hardware roscontrol_sot_hardware.cpp)

# Create directory for local wrap.so
file(MAKE_DIRECTORY
  ${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/test/sot_test_device)


pkg_config_use_dependency(roscontrol_sot_hardware controller_manager
  NO_INCLUDE_SYSTEM)

target_link_libraries(roscontrol_sot_hardware
  ${catkin_LIBRARIES}
  ${bullet_libraries}
  )

# Add the library to wrap the test device.
MACRO(build_test_device)

  SET(DEVICE_NAME sot-test-device)
  ADD_LIBRARY(${DEVICE_NAME}
    SHARED
    sot-test-device.cpp
  )

  set_target_properties(${DEVICE_NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  #set_target_properties(${DEVICE_NAME} PROPERTIES PREFIX "")

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "sot-core")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "pinocchio")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic_graph_bridge")

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${DEVICE_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${DEVICE_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/test/${PYTHON_LIBRARY_NAME}"
    ${DEVICE_NAME}
    sot/test/${PYTHON_LIBRARY_NAME}/wrap
    )

ENDMACRO()

build_test_device()

# Add the library to wrap the controller of Test.
MACRO(build_test_controller)

  SET(CONTROLLER_NAME sot-test-controller)
  ADD_LIBRARY(${CONTROLLER_NAME}
    SHARED
    sot-test-controller.cpp
  )

  #
  set_target_properties(${CONTROLLER_NAME}
    PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  #set_target_properties(${CONTROLLER_NAME} PROPERTIES PREFIX "")

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")

  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic_graph_bridge")

  ADD_DEPENDENCIES(${CONTROLLER_NAME} sot-test-device)
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_bridge")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_interpreter")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "sot-test-device")
 
  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

ENDMACRO()

build_test_controller()

