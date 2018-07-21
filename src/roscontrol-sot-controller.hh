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
#include <ros/ros.h>
#include <control_toolbox/pid.h>

/** URDF DOM*/
#include <urdf_parser/urdf_parser.h>

/* Local header */
#include "log.hh"

namespace sot_controller 
{
  enum SotControlMode { POSITION, EFFORT};

  class XmlrpcHelperException : public ros::Exception
  {
  public:
    XmlrpcHelperException(const std::string& what)
      : ros::Exception(what) {}
  };

  
  struct EffortControlPDMotorControlData
  {
    control_toolbox::Pid pid_controller;

    //double p_gain,d_gain,i_gain;
    double prev;
    double vel_prev;
    double des_pos;
    double integ_err;

    EffortControlPDMotorControlData();
    //    void read_from_xmlrpc_value(XmlRpc::XmlRpcValue &aXRV);
    void read_from_xmlrpc_value(const std::string &prefix);
  };

#ifndef CONTROLLER_INTERFACE_KINETIC
  typedef std::set<std::string> ClaimedResources;
#endif 
  /**
     This class encapsulates the Stack of Tasks inside the ros-control infra-structure.
     
   */
  namespace lhi = hardware_interface;
  namespace lci = controller_interface;
  
  class RCSotController : public lci::ControllerBase,
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
    std::vector<lhi::JointHandle> joints_;
    std::vector<std::string> joints_name_;

    /// \brief Vector towards the IMU.
    std::vector<lhi::ImuSensorHandle> imu_sensor_;

    /// \brief Vector of 6D force sensor.
    std::vector<lhi::ForceTorqueSensorHandle> ft_sensors_;
    
    /// \brief Vector of temperature sensors for the actuators.
    std::vector<lhi::ActuatorTemperatureSensorHandle> 
    act_temp_sensors_;
    
    /// \brief Interface to the joints controlled in position.
    lhi::PositionJointInterface * pos_iface_;

    /// \brief Interface to the joints controlled in force.
    lhi::EffortJointInterface * effort_iface_;
    
    /// \brief Interface to the sensors (IMU).
    lhi::ImuSensorInterface* imu_iface_;

    /// \brief Interface to the sensors (Force).
    lhi::ForceTorqueSensorInterface* ft_iface_;
    
    /// \brief Interface to the actuator temperature sensor.
    lhi::ActuatorTemperatureSensorInterface  * act_temp_iface_;

    /// @}

    /// \brief Log
    rc_sot_system::Log RcSotLog;
    /// @}
    
    const std::string type_name_;

    /// \brief Adapt the interface to Gazebo simulation
    bool simulation_mode_;

    /// \brief The robot can controlled in effort or position mode (default).
    SotControlMode control_mode_;

   
    /// \brief Implement a PD controller for the robot when the dynamic graph
    /// is not on.
    std::map<std::string, EffortControlPDMotorControlData> effort_mode_pd_motors_;

    /// \brief Give the desired position when the dynamic graph is not on.
    std::map<std::string, double> desired_init_pose_;
    
    /// \brief Map from ros-control quantities to robot device
    /// ros-control quantities are for the sensors:
    /// * motor-angles
    /// * joint-angles
    /// * velocities
    /// * torques
    /// ros-control quantities for control are:
    /// * joints
    /// * torques
    std::map<std::string,std::string> mapFromRCToSotDevice_;

    /// To be able to subsample control period.
    double accumulated_time_;

    /// Jitter for the subsampling.
    double jitter_;

    /// URDF model of the robot.
    urdf::ModelInterfaceSharedPtr modelURDF_;    

  public :

    RCSotController ();

    /// \brief Read the configuration files, 
    /// claims the request to the robot and initialize the Stack-Of-Tasks.
    bool initRequest (lhi::RobotHW * robot_hw, 
		      ros::NodeHandle &robot_nh,
		      ros::NodeHandle &controller_nh,
		      ClaimedResources & claimed_resources);

    /// \brief Display claimed resources
    void displayClaimedResources(ClaimedResources & claimed_resources);

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
    bool initInterfaces(lhi::RobotHW * robot_hw,
			ros::NodeHandle &,
			ros::NodeHandle &,
			ClaimedResources & claimed_resources);

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

    /// \brief Read the PID information of the robot in effort mode.
    bool readParamsEffortControlPDMotorControlData(ros::NodeHandle &robot_nh);

    /// \brief Read the desired initial pose of the robot in position mode.
    bool readParamsPositionControlData(ros::NodeHandle &robot_nh);

    /// \brief Read the control period.
    bool readParamsdt(ros::NodeHandle & robot_nh);
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
    
    ///@{ Control the robot while waiting for the SoT
    /// Default control in effort.
    void localStandbyEffortControlMode(const ros::Duration& period);
    /// Default control in position.
    void localStandbyPositionControlMode();
    
    ///@}
    /// Extract control values to send to the simulator.
    void readControl(std::map<std::string,dgs::ControlValues> &controlValues);

    /// Map of sensor readings
    std::map <std::string,dgs::SensorValues> sensorsIn_;

    /// Map of control values
    std::map<std::string,dgs::ControlValues> controlValues_;

    /// Control period
    double dt_;
    
    /// \brief Command send to motors
    /// Depending on control_mode it can be either
    /// position control or torque control.
    std::vector<double> command_;
    
    /// One iteration: read sensor, compute the control law, apply control.
    void one_iteration();

    /// Read URDF model from /robot_description parameter.
    bool readUrdf(ros::NodeHandle &robot_nh);
  };
}

#endif /* RC_SOT_CONTROLLER_H */
