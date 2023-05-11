/*
 * Copyright 2016,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 *
 */

#include <pinocchio/fwd.hpp>

// include pinocchio before boost

#include <ros/console.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

#include "sot-test-controller.hh"

#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

const std::string SoTTestController::LOG_PYTHON =
    "/tmp/TestController_python.out";

using namespace std;

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

void workThread(SoTTestController *aSoTTest) {
  dynamicgraph::Interpreter aLocalInterpreter(
      dynamicgraph::rosInit(false, true));

  aSoTTest->interpreter_ =
      boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);
  std::cout << "Going through the thread." << std::endl;
  {
    boost::lock_guard<boost::mutex> lock(mut);
    data_ready = true;
  }
  cond.notify_all();
  ros::waitForShutdown();
}

SoTTestController::SoTTestController(std::string RobotName)
    : device_(new SoTTestDevice(RobotName)) {
  init();
}

SoTTestController::SoTTestController(const char robotName[])
    : device_(new SoTTestDevice(robotName)) {
  init();
}

void SoTTestController::init() {
  std::cout << "Going through SoTTestController." << std::endl;

  boost::thread thr(workThread, this);
  sotDEBUG(25) << __FILE__ << ":" << __FUNCTION__ << "(#" << __LINE__ << " )"
               << std::endl;

  boost::unique_lock<boost::mutex> lock(mut);
  cond.wait(lock);
}

SoTTestController::~SoTTestController() {}

void SoTTestController::setupSetSensors(
    map<string, dgsot::SensorValues> &SensorsIn) {
  device_->setupSetSensors(SensorsIn);
}

void SoTTestController::nominalSetSensors(
    map<string, dgsot::SensorValues> &SensorsIn) {
  device_->nominalSetSensors(SensorsIn);
}

void SoTTestController::cleanupSetSensors(
    map<string, dgsot::SensorValues> &SensorsIn) {
  device_->cleanupSetSensors(SensorsIn);
}

void SoTTestController::getControl(
    map<string, dgsot::ControlValues> &controlOut, const double &period) {
  try {
    sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
    device_->getControl(controlOut, period);
    sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
  } catch (dynamicgraph::sot::ExceptionAbstract &err) {
    std::cout << __FILE__ << " " << __FUNCTION__ << " (" << __LINE__ << ") "
              << err.getStringMessage() << endl;
    throw err;
  }
}

void SoTTestController::setControlSize(const int &size) {
  device_->setControlSize(size);
}

void SoTTestController::initialize() {}

void SoTTestController::setNoIntegration(void) { device_->setNoIntegration(); }

void SoTTestController::setSecondOrderIntegration(void) {
  device_->setSecondOrderIntegration();
}

void SoTTestController::runPython(std::ostream &file,
                                  const std::string &command,
                                  dynamicgraph::Interpreter &interpreter) {
  file << ">>> " << command << std::endl;
  std::string lerr(""), lout(""), lres("");
  interpreter.runCommand(command, lres, lout, lerr);
  if (lres != "None") {
    if (lres == "<NULL>") {
      file << lout << std::endl;
      file << "------" << std::endl;
      file << lerr << std::endl;
    } else
      file << lres << std::endl;
  }
}

void SoTTestController::startupPython() {
  std::ofstream aof(LOG_PYTHON.c_str());
  runPython(aof, "import sys, os", *interpreter_);
  runPython(aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
  runPython(aof, "path = []", *interpreter_);
  runPython(aof,
            "for p in pythonpath.split(':'):\n"
            "  if p not in sys.path:\n"
            "    path.append(p)",
            *interpreter_);
  runPython(aof, "path.extend(sys.path)", *interpreter_);
  runPython(aof, "sys.path = path", *interpreter_);

  // Calling again rosInit here to start the spinner. It will
  // deal with topics and services callbacks in a separate, non
  // real-time thread. See roscpp documentation for more
  // information.
  dynamicgraph::rosInit(true);
  aof.close();
}

extern "C" {
dgsot::AbstractSotExternalInterface *createSotExternalInterface() {
  return new SoTTestController("test_robot");
}
}
