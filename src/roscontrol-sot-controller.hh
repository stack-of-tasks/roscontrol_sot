/*
 * Author: Olivier STASSE
 */

#ifndef RC_SOT_CONTROLLER_H
#define RC_SOT_CONTROLLER_H

#include <string>
#include <map>

#pragma GCC diagnostic push
#pragma GCC system_header
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pal_hardware_interfaces/actuator_temperature_interface.h>
#include <pluginlib/class_list_macros.h>
#pragma GCC diagnostic pop

#include <dynamic_graph_bridge/sot_loader_basic.hh>
#include <ros/ros.h>
#include <control_toolbox/pid.h>

/** URDF DOM*/
#include <urdf_parser/urdf_parser.h>

/* Local header */
#include "log.hh"

namespace sot_controller
{
  enum ControlMode { POSITION, VELOCITY, EFFORT};
  namespace lhi = hardware_interface;
  namespace lci = controller_interface;

  class XmlrpcHelperException : public ros::Exception
  {
  public:
    XmlrpcHelperException(const std::string& what)
      : ros::Exception(what) {}
  };


  struct ControlPDMotorControlData
  {
    control_toolbox::Pid pid_controller;

    ControlPDMotorControlData();
    //    void read_from_xmlrpc_value(XmlRpc::XmlRpcValue &aXRV);
    void read_from_xmlrpc_value(const std::string &prefix);
  };

  struct JointSotHandle
  {
    lhi::JointHandle joint;
    double desired_init_pose;
    ControlMode sot_control_mode;
    ControlMode ros_control_mode;
  };

  typedef std::map<std::string,JointSotHandle>::iterator it_joint_sot_h;
#ifndef CONTROLLER_INTERFACE_KINETIC
  typedef std::set<std::string> ClaimedResources;
#endif
  /**
     This class encapsulates the Stack of Tasks inside the ros-control infra-structure.

   */
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
    std::map<std::string,JointSotHandle> joints_;
    std::vector<std::string> joints_name_;

    /// \brief Vector towards the IMU.
    std::vector<lhi::ImuSensorHandle> imu_sensor_;

    /// \brief Vector of 6D force sensor.
    std::vector<lhi::ForceTorqueSensorHandle> ft_sensors_;

#ifdef TEMPERATURE_SENSOR_CONTROLLER
    /// \brief Vector of temperature sensors for the actuators.
    std::vector<lhi::ActuatorTemperatureSensorHandle>
    act_temp_sensors_;
#endif

    /// \brief Interface to the joints controlled in position.
    lhi::PositionJointInterface * pos_iface_;

    /// \brief Interface to the joints controlled in force.
    lhi::EffortJointInterface * effort_iface_;

    /// \brief Interface to the sensors (IMU).
    lhi::ImuSensorInterface* imu_iface_;

    /// \brief Interface to the sensors (Force).
    lhi::ForceTorqueSensorInterface* ft_iface_;

#ifdef TEMPERATURE_SENSOR_CONTROLLER
    /// \brief Interface to the actuator temperature sensor.
    lhi::ActuatorTemperatureSensorInterface  * act_temp_iface_;
#endif
    /// @}

    /// \brief Log
    rc_sot_system::Log RcSotLog_;
    /// @}

    const std::string type_name_;

    /// \brief Adapt the interface to Gazebo simulation
    bool simulation_mode_;

    /// \brief Implement a PD controller for the robot when the dynamic graph
    /// is not on.
    std::map<std::string, ControlPDMotorControlData> effort_mode_pd_motors_;

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

    /// \brief Verbosity level for ROS messages during initRequest/initialization phase.
    /// 0: no messages or error 1: info 2: debug
    int verbosity_level_;

    /// URDF model of the robot.
    urdf::ModelInterfaceSharedPtr modelURDF_;

    /// Profile log
    rc_sot_system::ProfileLog profileLog_;

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

    /// Initialize internal structure for the logs based on nbDofs
    /// number of force sensors and size of the buffer.
    void initLogs();

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

    /// \brief Read verbosity level to display messages mostly during initialization
    void readParamsVerbosityLevel(ros::NodeHandle &robot_nh);
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

    /// Returns control mode by reading rosparam.
    /// It reads /sot_controller/control_mode/joint_name
    /// and check 
    bool
    getJointControlMode(std::string &joint_name,
			JointSotHandle &aJointSotHandle);
  };
}

#endif /* RC_SOT_CONTROLLER_H */
