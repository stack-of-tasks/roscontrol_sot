/*
   Olivier Stasse CNRS,
   31/07/2012
   Object to log the low-level informations of a robot.
*/
#include "log.hh"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/time.h>

using namespace std;
using namespace rc_sot_system;

DataToLog::DataToLog() {}

void DataToLog::init(ProfileLog &aProfileLog) {
  profileLog_ = aProfileLog;
  motor_angle.resize(aProfileLog.nbDofs * aProfileLog.length);
  joint_angle.resize(aProfileLog.nbDofs * aProfileLog.length);
  velocities.resize(aProfileLog.nbDofs * aProfileLog.length);
  torques.resize(aProfileLog.nbDofs * aProfileLog.length);
  motor_currents.resize(aProfileLog.nbDofs * aProfileLog.length);
  orientation.resize(4 * aProfileLog.length);
  accelerometer.resize(3 * aProfileLog.length);
  gyrometer.resize(3 * aProfileLog.length);
  force_sensors.resize(aProfileLog.nbForceSensors * 6 * aProfileLog.length);
  temperatures.resize(aProfileLog.nbDofs * aProfileLog.length);
  controls.resize(aProfileLog.nbDofs * aProfileLog.length);
  timestamp.resize(aProfileLog.length);
  duration.resize(aProfileLog.length);

  for (unsigned int i = 0; i < aProfileLog.nbDofs * aProfileLog.length; i++) {
    motor_angle[i] = joint_angle[i] = velocities[i] = 0.0;
  }
}

Log::Log() : lref_(0), lrefts_(0) {}

void Log::init(ProfileLog &aProfileLog) {
  profileLog_ = aProfileLog;
  lref_ = 0;
  lrefts_ = 0;
  StoredData_.init(aProfileLog);
  struct timeval current;
  gettimeofday(&current, 0);

  timeorigin_ = (double)current.tv_sec + 0.000001 * ((double)current.tv_usec);
}

void Log::record(DataToLog &aDataToLog) {
  if ((aDataToLog.motor_angle.size() != profileLog_.nbDofs) ||
      (aDataToLog.velocities.size() != profileLog_.nbDofs))
    return;

  for (unsigned int JointID = 0; JointID < aDataToLog.nbDofs(); JointID++) {
    if (aDataToLog.motor_angle.size() > JointID)
      StoredData_.motor_angle[JointID + lref_] =
          aDataToLog.motor_angle[JointID];
    if (aDataToLog.joint_angle.size() > JointID)
      StoredData_.joint_angle[JointID + lref_] =
          aDataToLog.joint_angle[JointID];
    if (aDataToLog.velocities.size() > JointID)
      StoredData_.velocities[JointID + lref_] = aDataToLog.velocities[JointID];
    if (aDataToLog.torques.size() > JointID)
      StoredData_.torques[JointID + lref_] = aDataToLog.torques[JointID];
    if (aDataToLog.motor_currents.size() > JointID)
      StoredData_.motor_currents[JointID + lref_] =
          aDataToLog.motor_currents[JointID];
    if (aDataToLog.temperatures.size() > JointID)
      StoredData_.temperatures[JointID + lref_] =
          aDataToLog.temperatures[JointID];
    if (aDataToLog.controls.size() > JointID)
      StoredData_.controls[JointID + lref_] = aDataToLog.controls[JointID];
  }
  for (unsigned int axis = 0; axis < 3; axis++) {
    StoredData_.accelerometer[lrefts_ * 3 + axis] =
        aDataToLog.accelerometer[axis];
    StoredData_.gyrometer[lrefts_ * 3 + axis] = aDataToLog.gyrometer[axis];
  }
  std::size_t width_pad = 6 * profileLog_.nbForceSensors;

  for (unsigned int fsID = 0; fsID < profileLog_.nbForceSensors; fsID++) {
    for (unsigned int axis = 0; axis < 6; axis++) {
      StoredData_.force_sensors[lrefts_ * width_pad + fsID * 6 + axis] =
          aDataToLog.force_sensors[fsID * 6 + axis];
    }
  }

  struct timeval current;
  gettimeofday(&current, 0);

  StoredData_.timestamp[lrefts_] =
      ((double)current.tv_sec + 0.000001 * (double)current.tv_usec) -
      timeorigin_;

  StoredData_.duration[lrefts_] = time_stop_it_ - time_start_it_;

  lref_ += profileLog_.nbDofs;
  lrefts_++;
  if (lref_ >= profileLog_.nbDofs * profileLog_.length) {
    lref_ = 0;
    lrefts_ = 0;
  }
  assert(lref_ == lrefts_*profileLog_.nbDofs);
}

void Log::start_it() {
  struct timeval current;
  gettimeofday(&current, 0);

  time_start_it_ =
      ((double)current.tv_sec + 0.000001 * (double)current.tv_usec) -
      timeorigin_;
}

void Log::stop_it() {
  struct timeval current;
  gettimeofday(&current, 0);

  time_stop_it_ =
      ((double)current.tv_sec + 0.000001 * (double)current.tv_usec) -
      timeorigin_;
}

void Log::save(std::string &fileName) {
  assert(lref_ == lrefts_*profileLog_.nbDofs);

  std::string suffix("-mastate.log");
  saveVector(fileName, suffix, StoredData_.motor_angle, profileLog_.nbDofs, lref_);
  suffix = "-jastate.log";
  saveVector(fileName, suffix, StoredData_.joint_angle, profileLog_.nbDofs, lref_);
  suffix = "-vstate.log";
  saveVector(fileName, suffix, StoredData_.velocities, profileLog_.nbDofs, lref_);
  suffix = "-torques.log";
  saveVector(fileName, suffix, StoredData_.torques, profileLog_.nbDofs, lref_);
  suffix = "-motor-currents.log";
  saveVector(fileName, suffix, StoredData_.motor_currents, profileLog_.nbDofs, lref_);
  suffix = "-accelero.log";
  saveVector(fileName, suffix, StoredData_.accelerometer, 3, 3*lrefts_);
  suffix = "-gyro.log";
  saveVector(fileName, suffix, StoredData_.gyrometer, 3, 3*lrefts_);

  ostringstream oss;
  oss << "-forceSensors.log";
  suffix = oss.str();
  saveVector(fileName, suffix, StoredData_.force_sensors,
             6 * profileLog_.nbForceSensors,
             6 * profileLog_.nbForceSensors * lrefts_);

  suffix = "-temperatures.log";
  saveVector(fileName, suffix, StoredData_.temperatures, profileLog_.nbDofs,
      lref_);

  suffix = "-controls.log";
  saveVector(fileName, suffix, StoredData_.controls, profileLog_.nbDofs, lref_);

  suffix = "-duration.log";
  saveVector(fileName, suffix, StoredData_.duration, 1, lrefts_);
}

inline void writeHeaderToBinaryBuffer(ofstream &of, const std::size_t &nVector,
                                      const std::size_t &vectorSize) {
  of.write((const char *)(&nVector), sizeof(std::size_t));
  of.write((const char *)(&vectorSize), sizeof(std::size_t));
}

inline void writeToBinaryFile(ofstream &of, const double &t, const double &dt,
                              const std::vector<double> &data,
                              const std::size_t &idx, const std::size_t &size) {
  of.write((const char *)&t, sizeof(double));
  of.write((const char *)&dt, sizeof(double));
  of.write((const char *)(&data[idx]), size * (sizeof(double)));
}

void Log::saveVector(std::string &fileName, std::string &suffix,
                     const std::vector<double> &avector, std::size_t size,
                     std::size_t start) {
  ostringstream oss;
  oss << fileName;
  oss << suffix.c_str();
  std::string actualFileName = oss.str();

  ofstream aof(actualFileName.c_str(), std::ios::binary | std::ios::trunc);

  std::size_t idx = 0;
  double dt;
  if (aof.is_open()) {
    std::size_t N = size * profileLog_.length;

    writeHeaderToBinaryBuffer(aof, profileLog_.length, size + 2);
    for (unsigned long int i = 0; i < profileLog_.length; i++) {
      std::size_t k = (start + i) % profileLog_.length;

      // Compute and save dt
      if (i == 0) {
        dt = 0;
      } else if (k == 0) {
        dt = StoredData_.timestamp[0] -
             StoredData_.timestamp[profileLog_.length - 1];
      } else
        dt = StoredData_.timestamp[k] - StoredData_.timestamp[k - 1];
      writeToBinaryFile(aof, StoredData_.timestamp[i], dt, avector, idx, size);
      idx = (idx + size) % N;
    }
    aof.close();
    ROS_INFO_STREAM("Wrote log file " << actualFileName);
  }
}
