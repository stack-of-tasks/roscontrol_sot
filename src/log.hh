/* 
   Olivier Stasse CNRS,
   19/12/2016
   Object to control the low-level part of TALOS.
*/

#ifndef _RC_SOT_SYSTEM_LOG_H_
#define _RC_SOT_SYSTEM_LOG_H_

#include <vector>
#include <string>

namespace rc_sot_system {

  struct DataToLog
  {
    // Measured angle values at the motor side.
    std::vector<double> motor_angle;
    // Measured angle at the joint side.
    std::vector<double> joint_angle;
    // Measured or computed velocities.
    std::vector<double> velocities;
    // Measured torques.
    std::vector<double> torques;
    // Reconstructed orientation (from internal IMU).
    std::vector<double> orientation;
    // Measured linear acceleration
    std::vector<double> accelerometer;
    // Measured angular velocities
    std::vector<double> gyrometer;
    // Measured force sensors
    std::vector<double> force_sensors;
    // Measured motor currents
    std::vector<double> motor_currents;
    // Measured temperatures
    std::vector<double> temperatures;

    // Timestamp
    std::vector<double> timestamp;
    // Duration
    std::vector<double> duration;

    DataToLog();
    void init(unsigned int nbDofs, long int length);


  };

  class Log
  {
  private:
    // Actuated informations logged.
    unsigned int nbDofs_;
    // Number of iterations to be logged.
    unsigned int length_;

    // Current position in the circular buffer for angles.
    long unsigned int lref_;
    // Current position int the circular buffer for timestamp
    // lref_ = lrefts_ * nbDofs_
    long unsigned int lrefts_;

    // Circular buffer for all the data.
    DataToLog StoredData_;

    double timeorigin_;
    double time_start_it_;
    double time_stop_it_;

    // Save one vector of information.
    void saveVector(std::string &filename, 
		    std::string &suffix,
		    const std::vector<double> &avector,
		    unsigned int);

  public:
  
    Log();

    void init(unsigned int nbDofs, unsigned int length);
    void record(DataToLog &aDataToLog);

    void save(std::string &fileName);
    void start_it();
    void stop_it();

  };
}
#endif /* _RC_SOT_SYSTEM_LOG_H_ */
