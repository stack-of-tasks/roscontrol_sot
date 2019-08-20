/*
 * Copyright 2019,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 *
 */

#ifndef _SOT_TestDevice_H_
#define _SOT_TestDevice_H_


#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/matrix-geometry.hh>

namespace dgsot=dynamicgraph::sot;
namespace dg=dynamicgraph;

class SoTTestDevice: public 
dgsot::Device
{
 public:

  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string& getClassName () const		
  {  
    return CLASS_NAME;							    
  }
  
  SoTTestDevice(std::string RobotName);
  virtual ~SoTTestDevice();
  
  void setSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void timeStep(double ts) { timestep_ =ts;}

protected:

  /// \brief Previous robot configuration.
  dg::Vector previousState_;
  
  /// Intermediate variables to avoid allocation during control
  std::vector<double> baseff_;

  /// Accelerations measured by accelerometers
  dynamicgraph::Signal <dg::Vector, int> accelerometerSOUT_;
  /// Rotation velocity measured by gyrometers
  dynamicgraph::Signal <dg::Vector, int> gyrometerSOUT_;
  /// motor currents
  dynamicgraph::Signal <dg::Vector, int> currentsSOUT_;
  /// joint angles
  dynamicgraph::Signal <dg::Vector, int> joint_anglesSOUT_;
  /// motor angles
  dynamicgraph::Signal <dg::Vector, int> motor_anglesSOUT_;

  /// proportional and derivative position-control gains
  dynamicgraph::Signal <dg::Vector, int> p_gainsSOUT_;

  dynamicgraph::Signal <dg::Vector, int> d_gainsSOUT_;

  /// Protected methods for internal variables filling
  void setSensorsForce
  (std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsIMU
  (std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsEncoders
  (std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsVelocities
  (std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsTorquesCurrents
  (std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsGains
  (std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  
  /// Intermediate variables to avoid allocation during control
  dg::Vector dgforces_;
  dg::Vector dgRobotState_; // motor-angles
  dg::Vector joint_angles_; // joint-angles
  dg::Vector motor_angles_; // motor-angles
  dg::Vector dgRobotVelocity_; // motor velocities
  dg::Vector velocities_; // motor velocities
  dgsot::MatrixRotation pose;
  dg::Vector accelerometer_;
  dg::Vector gyrometer_;
  dg::Vector torques_;
  dg::Vector currents_;
  dg::Vector p_gains_;
  dg::Vector d_gains_;
};
#endif /* _SOT_TestDevice_H_*/
