#include <cmath>

#include "../src/log.hh"

int main() {
  rc_sot_system::DataToLog DataOneIter;
  rc_sot_system::ProfileLog profileLog;
  rc_sot_system::Log RcSotLog;
  std::size_t nbDofs(6);

  profileLog.nbDofs = nbDofs;
  profileLog.length = 1;
  profileLog.nbForceSensors = 0;
  DataOneIter.init(profileLog);
  // Length of the buffer
  profileLog.length = 100;
  RcSotLog.init(profileLog);

  for (std::size_t i = 0; i < 250; ++i) {
    for (std::size_t j = 0; j < nbDofs; ++j) {
      DataOneIter.motor_angle[j] = DataOneIter.joint_angle[j] =
          DataOneIter.velocities[j] = DataOneIter.torques[j] =
              DataOneIter.motor_currents[j] = DataOneIter.temperatures[j] =
                  DataOneIter.controls[j] = (double)(i + j);
    }
    RcSotLog.record(DataOneIter);
  }
  std::string filename("./test.log");
  RcSotLog.save(filename);
}
