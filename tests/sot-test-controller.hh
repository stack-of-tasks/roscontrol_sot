/*
 * Copyright 2016,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 *
 */

#ifndef _SOT_TestController_H_
#define _SOT_TestController_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal.h>

#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/device.hh>

#include "sot-test-device.hh"
namespace dgsot = dynamicgraph::sot;

class SoTTestController : public dgsot::AbstractSotExternalInterface {
 public:
  static const std::string LOG_PYTHON;

  SoTTestController();
  SoTTestController(const char robotName[]);
  SoTTestController(std::string robotName);
  virtual ~SoTTestController();

  void setupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut,
                  const double &period);

  void setControlSize(const int& size);
  void initialize();
  void setNoIntegration(void);
  void setSecondOrderIntegration(void);

  /// Embedded python interpreter accessible via Corba/ros
  boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);

  void runPython(std::ostream &file, const std::string &command,
                 dynamicgraph::Interpreter &interpreter);

  virtual void startupPython();

  void init();

  SoTTestDevice *device_;
};

#endif /* _SOT_TestController_H_ */
