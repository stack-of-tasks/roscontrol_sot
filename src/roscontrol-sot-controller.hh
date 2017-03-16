///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, CNRS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Olivier STASSE
 */

#ifndef RC_SOT_CONTROLLER_H
#define RC_SOT_CONTROLLER_H

#include <string>
#include <map>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pal_hardware_interfaces/actuator_temperature_interface.h>

#include <dynamic_graph_bridge/sot_loader_basic.hh>

#include "log.hh"

namespace rc_sot_controller 
{
  enum SotControlMode { POSITION, EFFORT};

  /**
     This class encapsulates the Stack of Tasks inside the ros-control infra-structure.
     
   */
  class RCSotController : public controller_interface::ControllerBase,
			       SotLoaderBasic
  {
    
  protected:
    /// Robot nb dofs.
    size_t nbDofs_;
    
    /// Data log.
    rc_sot_system::DataToLog DataOneIter_;
  private:
   
    /// @{ \name Ros-control related fields
    
    /// \brief Vector of joint handles.
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<std::string> joints_name_;

    /// \brief Vector towards the IMU.
    std::vector<hardware_interface::ImuSensorHandle> imu_sensor_;

    /// \brief Vector of 6D force sensor.
    std::vector<hardware_interface::ForceTorqueSensorHandle> ft_sensors_;
    
    /// \brief Vector of temperature sensors for the actuators.
    std::vector<hardware_interface::ActuatorTemperatureSensorHandle> 
    act_temp_sensors_;
    
    /// \brief Interface to the joints controlled in position.
    hardware_interface::PositionJointInterface * pos_iface_;

    /// \brief Interface to the joints controlled in force.
    hardware_interface::EffortJointInterface * effort_iface_;
    
    /// \brief Interface to the sensors (IMU).
    hardware_interface::ImuSensorInterface* imu_iface_;

    /// \brief Interface to the sensors (Force).
    hardware_interface::ForceTorqueSensorInterface* ft_iface_;
    
    /// \brief Interface to the actuator temperature sensor.
    hardware_interface::ActuatorTemperatureSensorInterface  * act_temp_iface_;

    /// @}

    /// \brief Log
    rc_sot_system::Log RcSotLog;
    /// @}
    
    const std::string type_name_;

    /// \brief Adapt the interface to Gazebo simulation
    bool simulation_mode_;

    /// \brief The robot can controlled in effort or position mode (default).
    SotControlMode control_mode_;
    
    /// \brief Map from ros-control quantities to robot device
    /// ros-control quantities are for the sensors:
    /// * motor-angles
    /// * joint-angles
    /// * velocities
    /// * torques
    /// ros-control quantities for control are:
    /// * joints
    /// * torques
    std::map<std::string,std::string> mapFromRCToSotDevice;

  public :

    RCSotController ();

    /// \brief Read the configuration files, claims the request to the robot and initialize the Stack-Of-Tasks.
    bool initRequest (hardware_interface::RobotHW * robot_hw, 
		      ros::NodeHandle &robot_nh,
		      ros::NodeHandle &controller_nh,
		      std::set<std::string> & claimed_resources);

    /// \brief Claims
    bool init();

    /// \brief Read the sensor values, calls the control graph, and apply the control.
    /// 
    void update(const ros::Time&, const ros::Duration& );
    /// \brief Starting by filling the sensors.
    void starting(const ros::Time&);
    /// \brief Stopping the control
    void stopping(const ros::Time&);
    /// \brief Display the kind of hardware interface that this controller is using.
    virtual std::string getHardwareInterfaceType() const;

  protected:
    /// Initialize the roscontrol interfaces
    bool initInterfaces(hardware_interface::RobotHW * robot_hw,
			ros::NodeHandle &,
			ros::NodeHandle &,
			std::set<std::string> & claimed_resources);

    /// Initialize the hardware interface using the joints.
    bool initJoints();
    /// Initialize the hardware interface accessing the IMU.
    bool initIMU();
    /// Initialize the hardware interface accessing the force sensors.
    bool initForceSensors();
    /// Initialize the hardware interface accessing the temperature sensors.
    bool initTemperatureSensors();

    ///@{ \name Read the parameter server
    /// \brief Entry point
    bool readParams(ros::NodeHandle &robot_nh);

    /// \brief Creates the list of joint names.
    bool readParamsJointNames(ros::NodeHandle &robot_nh);

    /// \brief Set the SoT library name.
    bool readParamsSotLibName(ros::NodeHandle &robot_nh);

    /// \Brief Set the mapping between ros-control and the robot device
    /// For instance the yaml file should have a line with map_rc_to_sot_device:
    ///   map_rc_to_sot_device: [ ]
    bool readParamsFromRCToSotDevice(ros::NodeHandle &robot_nh);
    
    /// \brief Read the control mode.
    bool readParamsControlMode(ros::NodeHandle & robot_nh);

    ///@}

    /// \brief Fill the SoT map structures
    void fillSensorsIn(std::string &title, std::vector<double> & data);

    /// \brief Get the information from the low level and calls fillSensorsIn.
    void fillJoints();
    
    /// In the map sensorsIn_ creates the key "name_IMUNb"
    /// and associate to this key the vector data.
    void setSensorsImu(std::string &name,
		       int IMUNb,
		       std::vector<double> &data);

    /// @{ \name Fill the sensors 
    /// Read the imus and set the interface to the SoT.
    void fillImu();
    /// Read the force sensors
    void fillForceSensors();
    /// Read the temperature sensors
    void fillTempSensors();
    /// Entry point for reading all the sensors .
    void fillSensors();
    ///@}
    

    /// Extract control values to send to the simulator.
    void readControl(std::map<std::string,dgs::ControlValues> &controlValues);

    /// Map of sensor readings
    std::map <std::string,dgs::SensorValues> sensorsIn_;

    /// Map of control values
    std::map<std::string,dgs::ControlValues> controlValues_;

    /// \brief Command send to motors
    /// Depending on control_mode it can be either
    /// position control or torque control.
    std::vector<double> command_;
    
    /// One iteration: read sensor, compute the control law, apply control.
    void one_iteration();

  };
}

#endif /* RC_SOT_CONTROLLER_H */
