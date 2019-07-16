#include <fstream>
#include <iomanip>
#include <dlfcn.h>
#include <sstream>


#include "roscontrol-sot-controller.hh"

#include<ros/console.h>

#define ENABLE_RT_LOG
#include<dynamic-graph/real-time-logger.h>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/rcoh2sot.dat"

#define RESETDEBUG5() { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::out);		\
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app);		\
    DebugFile << __FILE__ << ":"			\
	      << __FUNCTION__ << "(#"			\
	      << __LINE__ << "):" << x << std::endl;	\
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;		\
    DebugFile.close();}

#define RESETDEBUG4()
#define ODEBUG4FULL(x)
#define ODEBUG4(x)

class LoggerROSStream : public ::dynamicgraph::LoggerStream
{
  public:
    void write (const char* c) {
      ROS_ERROR ("%s",c);
    }
};

/// lhi: nickname for local_hardware_interface
/// Depends if we are on the real robot or not.

namespace lhi=hardware_interface;
using namespace lhi;

using namespace rc_sot_system;

namespace sot_controller  
{
  typedef std::map<std::string,std::string>::iterator it_map_rt_to_sot;
  typedef std::map<std::string,std::string>::iterator it_control_mode;
  
  ControlPDMotorControlData::ControlPDMotorControlData()
  {
  }
  
  void ControlPDMotorControlData::read_from_xmlrpc_value
  (const std::string &prefix)
  {
    pid_controller.initParam(prefix);
  }
   
  RCSotController::
  RCSotController():
    // Store 32 DoFs for 5 minutes (1 Khz: 5*60*1000)
    // -> 124 Mo of data.
    type_name_("RCSotController"),
    simulation_mode_(false),
    accumulated_time_(0.0),
    jitter_(0.0),
    verbosity_level_(0)
  {
    RESETDEBUG4();
    profileLog_.length=300000;
  }
  
  void RCSotController::
  displayClaimedResources(ClaimedResources & claimed_resources)
  {
#ifdef CONTROLLER_INTERFACE_KINETIC
    ClaimedResources::iterator it_claim;
    ROS_INFO_STREAM("Size of claimed resources: "<< claimed_resources.size());
    for (it_claim = claimed_resources.begin(); 
	 it_claim != claimed_resources.end(); 
	 ++it_claim)
      {
	hardware_interface::InterfaceResources & aclaim = *it_claim;
	ROS_INFO_STREAM("Claimed by RCSotController: " << aclaim.hardware_interface);
	
	for(std::set<std::string>::iterator
	      it_set_res=aclaim.resources.begin();
	    it_set_res!=aclaim.resources.end();
	    it_set_res++)
	  {
	    ROS_INFO_STREAM(" Resources belonging to the interface:" <<
			    *it_set_res);
	  }
	    
      }
#else
    std::set<std::string >::iterator it_claim;
    ROS_INFO_STREAM("Size of claimed resources: "<< claimed_resources.size());
    for (it_claim = claimed_resources.begin();
	 it_claim != claimed_resources.end();
	 ++it_claim)
      {
	std::string aclaim = *it_claim;
	ROS_INFO_STREAM("Claimed by RCSotController: " << aclaim);
      }
#endif
  }

  void RCSotController::initLogs()
  {
    ROS_INFO_STREAM("Initialize log data structure");
    /// Initialize the size of the data to store.
    /// Set temporary profileLog to one
    /// because DataOneIter is just for one iteration.
    size_t tmp_length = profileLog_.length;
    profileLog_.length = 1;
    DataOneIter_.init(profileLog_);

    /// Set profile Log to real good value for the stored data.
    profileLog_.length= tmp_length;
    /// Initialize the data logger for 300s.
    RcSotLog_.init(profileLog_);

  }
  
  bool RCSotController::
  initRequest (lhi::RobotHW * robot_hw,
	       ros::NodeHandle &robot_nh,
	       ros::NodeHandle &controller_nh,
	       ClaimedResources & claimed_resources)
  {
    /// Read the parameter server
    if (!readParams(robot_nh))
      return false;

    /// Create ros control interfaces to hardware
    /// Recalls: init() is called by initInterfaces()
    if (!initInterfaces(robot_hw,robot_nh,controller_nh,claimed_resources))
      return false;

    /// Create all the internal data structures for logging.
    initLogs();
    
    /// Create SoT
    SotLoaderBasic::Initialization();

    /// Fill desired position during the phase where the robot is waiting.
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
	std::string joint_name = joints_name_[idJoint];
	std::map<std::string,ControlPDMotorControlData>::iterator
	  search_ecpd = effort_mode_pd_motors_.find(joint_name);
	
	if (search_ecpd!=effort_mode_pd_motors_.end())
	  {
	    /// If we are in effort mode then the device should not do any integration.
	    sotController_->setNoIntegration();

	  }
      }
    return true;
  }
  
  bool RCSotController::
  initInterfaces(lhi::RobotHW * robot_hw,
		 ros::NodeHandle &,
		 ros::NodeHandle &,
		 ClaimedResources & claimed_resources)
  {
    std::string lns;
    lns="hardware_interface";

    // Check if construction finished cleanly
    if (state_!=CONSTRUCTED)
      {
	ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      }

    // Get a pointer to the joint position control interface
    pos_iface_ = robot_hw->get<PositionJointInterface>();
    if (!pos_iface_)
      {
	ROS_WARN("This controller did not find  a hardware interface of type PositionJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required."
		 , lns.c_str());
      }

    // Get a pointer to the joint velocity control interface
    vel_iface_ = robot_hw->get<VelocityJointInterface>();
    if (!vel_iface_)
      {
	ROS_WARN("This controller did not find  a hardware interface of type VelocityJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required."
		 , lns.c_str());
      }

    // Get a pointer to the joint effort control interface
    effort_iface_ = robot_hw->get<EffortJointInterface>();
    if (! effort_iface_)
      {
	ROS_WARN("This controller did not find a hardware interface of type EffortJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required.",
		 lns.c_str());
      }

    // Get a pointer to the force-torque sensor interface
    ft_iface_ = robot_hw->get<ForceTorqueSensorInterface>();
    if (! ft_iface_ )
      {
	ROS_WARN("This controller did not find a hardware interface of type '%s '. " 
		 " Make sure this is registered inthe %s::RobotHW class if it is required.",
		  internal :: demangledTypeName<ForceTorqueSensorInterface>().c_str(),lns.c_str());
      }
    
    // Get a pointer to the IMU sensor interface
    imu_iface_ = robot_hw->get<ImuSensorInterface>();
    if (! imu_iface_)
      {
	ROS_WARN("This controller did not find a hardware interface of type '%s'."
		 " Make sure this is registered in the %s::RobotHW class if it is required.",
		    internal :: demangledTypeName<ImuSensorInterface>().c_str(),lns.c_str());
      }

    // Temperature sensor not available in simulation mode
    if (!simulation_mode_)
      {
#ifdef TEMPERATURE_SENSOR_CONTROLLER
	// Get a pointer to the actuator temperature sensor interface
	act_temp_iface_ = robot_hw->get<ActuatorTemperatureSensorInterface>();
	if (!act_temp_iface_)
	  {
	    ROS_WARN("This controller did not find a hardware interface of type '%s'."
		     " Make sure this is registered in the %s::RobotHW class if it is required.",
		      internal :: demangledTypeName<ActuatorTemperatureSensorInterface>().c_str(),lns.c_str());
	  }
#endif
      }
	
      
    // Return which resources are claimed by this controller
    pos_iface_->clearClaims();
    vel_iface_->clearClaims();
    effort_iface_->clearClaims();
    
    if (! init())
      {
	ROS_ERROR("Failed to initialize sot-controller" );
	std :: cerr << "FAILED LOADING SOT CONTROLLER" << std::endl;
	return false ;
      }
    if (verbosity_level_>0)
      ROS_INFO_STREAM("Initialization of interfaces for sot-controller Ok !");

#ifdef CONTROLLER_INTERFACE_KINETIC
    hardware_interface::InterfaceResources iface_res;
    iface_res.hardware_interface =
      hardware_interface::internal::
      demangledTypeName<PositionJointInterface>();
    iface_res.resources = pos_iface_->getClaims();
    claimed_resources.push_back(iface_res);

    iface_res.hardware_interface =
      hardware_interface::internal::
      demangledTypeName<VelocityJointInterface>();
    iface_res.resources = vel_iface_->getClaims();
    claimed_resources.push_back(iface_res);

    iface_res.hardware_interface =
      hardware_interface::internal::
      demangledTypeName<EffortJointInterface>();
    iface_res.resources = effort_iface_->getClaims();
    claimed_resources.push_back(iface_res);

    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);

    pos_iface_->clearClaims();
    vel_iface_->clearClaims();
    effort_iface_->clearClaims();
#else
    claimed_resources = pos_iface_->getClaims();
    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);
    pos_iface_->clearClaims();

    claimed_resources = vel_iface_->getClaims();
    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);
    vel_iface_->clearClaims();

    claimed_resources = effort_iface_->getClaims();
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);
    effort_iface_->clearClaims();
#endif    
    if (verbosity_level_>0)
      ROS_INFO_STREAM("Initialization of sot-controller Ok !");
    // success
    state_ = INITIALIZED;

    return true;
  }
    
  bool RCSotController::
  init()
  {
    if (!initJoints()) 
      return false;
    if (!initIMU()) {
      ROS_WARN("could not initialize IMU sensor(s).");
    }
    if (!initForceSensors()) {
      ROS_WARN("could not initialize force sensor(s).");
    }
    if (!initTemperatureSensors()) {
      ROS_WARN("could not initialize temperature sensor(s).");
    }

    // Initialize ros node.
    int argc=1;
    char *argv[1];
    argv[0] = new char[10];
    strcpy(argv[0],"libsot");
    SotLoaderBasic::initializeRosNode(argc,argv);
    
    return true;
  }

  void RCSotController::
  readParamsVerbosityLevel(ros::NodeHandle &robot_nh)
  {
    if (robot_nh.hasParam("/sot_controller/verbosity_level"))
      {
	robot_nh.getParam("/sot_controller/verbosity_level",verbosity_level_);
	ROS_INFO_STREAM("Verbosity_level " << verbosity_level_);
      }
    if (robot_nh.hasParam("/sot_controller/log/size"))
      {
	int llength;
	robot_nh.getParam("/sot_controller/log/size",llength);
	profileLog_.length=(unsigned int)llength;
	ROS_INFO_STREAM("Size of the log " << profileLog_.length);
      }

  }
  
  bool RCSotController::
  readParamsSotLibName(ros::NodeHandle &robot_nh)
  {
    // Read param to find the library to load
    std::string dynamic_library_name;

   // Read libname
    if (!robot_nh.getParam("/sot_controller/libname",dynamic_library_name))
      {
    	ROS_ERROR_STREAM("Could not read param /sot_controller/libname");
    	if (robot_nh.hasParam("/sot_controller/libname")) 
    	  {
    	    ROS_ERROR_STREAM("Param /sot_controller/libname exists !");
    	  }
    	else
    	  {
    	    ROS_ERROR_STREAM("Param /sot_controller/libname does not exists !");
    	    return false;
    	  }
      }
    else
      {
    	if (verbosity_level_>0)
    	  ROS_INFO_STREAM("Loading library name: " << dynamic_library_name);
      }
    /// SotLoaderBasic related method calls.
    // Initialize the dynamic_library_name for the sotLoader
    setDynamicLibraryName(dynamic_library_name);
    return true;
  }

  bool RCSotController::
  readParamsPositionControlData(ros::NodeHandle &)
  {
    return false;
  }
  
  bool RCSotController::
  readParamsEffortControlPDMotorControlData(ros::NodeHandle &robot_nh)
  {
    // Read libname
    if (robot_nh.hasParam("/sot_controller/effort_control_pd_motor_init/gains"))
      {
       XmlRpc::XmlRpcValue xml_rpc_ecpd_init;
       robot_nh.getParamCached("/sot_controller/effort_control_pd_motor_init/gains",
                               xml_rpc_ecpd_init);

       /// Display gain during transition control.
       if (verbosity_level_>0)
    	 ROS_INFO("/sot_controller/effort_control_pd_motor_init/gains: %d %d %d\n",
    		  xml_rpc_ecpd_init.getType(),
		  XmlRpc::XmlRpcValue::TypeArray,
		  XmlRpc::XmlRpcValue::TypeStruct);
       
       effort_mode_pd_motors_.clear();
       
       for (size_t i=0;i<joints_name_.size();i++)
         {
	   // Check if the joint should be in ROS EFFORT mode
	   std::map<std::string,JointSotHandle>::iterator
	     search_joint_sot_handle = joints_.find(joints_name_[i]);
	   if (search_joint_sot_handle!=joints_.end())
	     {
	       JointSotHandle aJointSotHandle = search_joint_sot_handle->second;
	       if (aJointSotHandle.ros_control_mode==EFFORT)
		 {
		   // Test if PID data is present
		   if (xml_rpc_ecpd_init.hasMember(joints_name_[i]))
		     {
		       std::string prefix=
			 "/sot_controller/effort_control_pd_motor_init/gains/"
			 + joints_name_[i];
		       effort_mode_pd_motors_[joints_name_[i]].
			 read_from_xmlrpc_value(prefix);
		     }
		   else
		     ROS_ERROR("No PID data for effort controlled joint %s in /sot_controller/effort_control_pd_motor_init/gains/",
			       joints_name_[i].c_str());
		 }
	     }

         }
       return true;
      }
    
    ROS_ERROR("No parameter /sot_controller/effort_controler_pd_motor_init");
    return false;
  }
  
  bool RCSotController::
  readParamsVelocityControlPDMotorControlData(ros::NodeHandle &robot_nh)
  {
    // Read libname
    if (robot_nh.hasParam("/sot_controller/velocity_control_pd_motor_init/gains"))
      {
       XmlRpc::XmlRpcValue xml_rpc_ecpd_init;
       robot_nh.getParamCached("/sot_controller/velocity_control_pd_motor_init/gains",
                               xml_rpc_ecpd_init);

       /// Display gain during transition control.
       if (verbosity_level_>0)
    	 ROS_INFO("/sot_controller/velocity_control_pd_motor_init/gains: %d %d %d\n",
    		  xml_rpc_ecpd_init.getType(),
		  XmlRpc::XmlRpcValue::TypeArray,
		  XmlRpc::XmlRpcValue::TypeStruct);
       
       velocity_mode_pd_motors_.clear();
       
       for (size_t i=0;i<joints_name_.size();i++)
         {
	   // Check if the joint should be in ROS VELOCITY mode
	   std::map<std::string,JointSotHandle>::iterator
	     search_joint_sot_handle = joints_.find(joints_name_[i]);
	   if (search_joint_sot_handle!=joints_.end())
	     {
	       JointSotHandle aJointSotHandle = search_joint_sot_handle->second;
	       if (aJointSotHandle.ros_control_mode==VELOCITY)
		 {
		   // Test if PID data is present
		   if (xml_rpc_ecpd_init.hasMember(joints_name_[i]))
		     {
		       std::string prefix=
			 "/sot_controller/velocity_control_pd_motor_init/gains/"
			 + joints_name_[i];
		       velocity_mode_pd_motors_[joints_name_[i]].
			 read_from_xmlrpc_value(prefix);
		     }
		   else
		     ROS_ERROR("No PID data for velocity controlled joint %s in /sot_controller/velocity_control_pd_motor_init/gains/",
			       joints_name_[i].c_str());
		 }
	     }

         }
       return true;
      }
    
    ROS_ERROR("No parameter /sot_controller/velocity_controler_pd_motor_init");
    return false;
  }

  bool RCSotController::
  readParamsFromRCToSotDevice(ros::NodeHandle &robot_nh)
  {
    // Read libname
    if (robot_nh.hasParam("/sot_controller/map_rc_to_sot_device")) 
      {
    	if (robot_nh.getParam("/sot_controller/map_rc_to_sot_device",
    			      mapFromRCToSotDevice_))
    	  {
    	    /// TODO: Check if the mapping is complete wrt to the interface and the mapping.
    	    if (verbosity_level_>0)
    	      {
    		ROS_INFO_STREAM("Loading map rc to sot device: ");
    		for (it_map_rt_to_sot it = mapFromRCToSotDevice_.begin(); 
    		     it != mapFromRCToSotDevice_.end(); ++it) 
    		  ROS_INFO_STREAM( it->first << ", " << it->second);
    	      }
    	  }
    	else
    	  {
    	    ROS_ERROR_STREAM("Could not read param /sot_controller/map_rc_to_sot_device");
    	    return false;
    	  }
      }  
    else
      {
    	ROS_INFO_STREAM("Param /sot_controller/map_rc_to_sot_device does not exists !");
    	return false;
      }
    return true;
  }

  bool RCSotController::
  readParamsJointNames(ros::NodeHandle &robot_nh)
  {
    /// Check if the /sot_controller/joint_names parameter exists.
    if (robot_nh.hasParam("/sot_controller/joint_names")) 
      {
    	/// Read the joint_names list from this parameter
    	robot_nh.getParam("/sot_controller/joint_names",
    			  joints_name_);
    	for(std::vector<std::string>::size_type i=0;i<joints_name_.size();i++)
    	  {
    	    if (verbosity_level_>0)
    	      ROS_INFO_STREAM("joints_name_[" << i << "]=" << joints_name_[i]);

    	    if (modelURDF_.use_count())
    	      {
    		urdf::JointConstSharedPtr aJCSP = modelURDF_->getJoint(joints_name_[i]);
    		if (aJCSP.use_count()!=0)
    		  {
    		    if (verbosity_level_>0)
    		      ROS_INFO_STREAM( joints_name_[i] + " found in the robot model" );
    		  }
    		else
    		  {
    		    ROS_ERROR(" %s not found in the robot model",joints_name_[i].c_str());
    		    return false;
    		  }
    	      }
    	    else
    	      {
    		ROS_ERROR("No robot model loaded in /robot_description");
    		return false;
	      }
	  }
      }
    else
      return false;

    /// Deduce from this the degree of freedom number.
    nbDofs_ = joints_name_.size();
    profileLog_.nbDofs = nbDofs_;
	
    return true;
  }

  bool RCSotController::
  getJointControlMode(std::string &joint_name,
		      JointSotHandle &aJointSotHandle)
  {
    std::string scontrol_mode;
    static const std::string seffort("EFFORT"),svelocity("VELOCITY"),sposition("POSITION");
    static const std::string ros_control_mode = "ros_control_mode";

    /// Read the list of control_mode
    ros::NodeHandle rnh_ns("/sot_controller/control_mode/"+joint_name);
    
    ControlMode joint_control_mode;
    if (!rnh_ns.getParam(ros_control_mode,scontrol_mode))
      {
        ROS_ERROR("No %s for %s - We found %s",
    	      ros_control_mode.c_str(),
    	      joint_name.c_str(),
    	      scontrol_mode.c_str());
        return false;
      }
    
    if      (scontrol_mode==sposition)
      joint_control_mode=POSITION;
    else if (scontrol_mode==svelocity)
      joint_control_mode=VELOCITY;
    else if (scontrol_mode==seffort)
      joint_control_mode=EFFORT;
    else {
      ROS_ERROR("%s for %s not understood. Expected %s, %s or %s. Got %s",
    	    ros_control_mode.c_str(),
    	    joint_name.c_str(),
                sposition.c_str(),
                svelocity.c_str(),
                seffort.c_str(),
    	    scontrol_mode.c_str());
      return false;
    }
    
    aJointSotHandle.ros_control_mode = joint_control_mode;
    //aJointSotHandle.sot_control_mode = joint_control_mode;

    return true;
  }

  bool RCSotController::
  readParamsControlMode(ros::NodeHandle &robot_nh)
  {
    std::map<std::string,std::string> mapControlMode;
    
    // Read param from control_mode.
    if (robot_nh.hasParam("/sot_controller/control_mode")) 
      {
	/// For each listed joint
	for(unsigned int idJoint=0;idJoint<joints_name_.size();idJoint++)
	  {
	    std::string joint_name = joints_name_[idJoint];
	    JointSotHandle &aJoint = joints_[joint_name];
	    if (!getJointControlMode(joint_name,aJoint))
	      return false;
	    ROS_INFO("joint_name[%d]=%s, control_mode=%d",
                idJoint,joint_name.c_str(),aJoint.ros_control_mode);
	  }
      }
    else
      {
	ROS_INFO_STREAM("Default control mode : position");
      }
    /// Always return true;
    return true;
  }

  bool RCSotController::
  readParamsdt(ros::NodeHandle &robot_nh)
  {
    /// Reading the jitter is optional but it is a very good idea.
    if (robot_nh.hasParam("/sot_controller/jitter"))
      {
	robot_nh.getParam("/sot_controller/jitter",jitter_);
	if (verbosity_level_>0)
	  ROS_INFO_STREAM("jitter: " << jitter_);
      }

    /// Read /sot_controller/dt to know what is the control period
    if (robot_nh.hasParam("/sot_controller/dt"))
      {
	robot_nh.getParam("/sot_controller/dt",dt_);
	if (verbosity_level_>0)
	  ROS_INFO_STREAM("dt: " << dt_);
	return true;
      }

    ROS_ERROR("You need to define a control period in param /sot_controller/dt");
    return false;
  }

  bool RCSotController::
  readUrdf(ros::NodeHandle &robot_nh)
  {
    /// Reading the parameter /robot_description which contains the robot
    /// description
    if (!robot_nh.hasParam("/robot_description"))
      {
	ROS_ERROR("ROS application does not have robot_description");
	return false;
      }
    std::string robot_description_str;
    
    robot_nh.getParam("/robot_description",robot_description_str);

    modelURDF_ = urdf::parseURDF(robot_description_str);
    if (verbosity_level_>0)
      ROS_INFO("Loaded /robot_description %ld",modelURDF_.use_count());
    return true;
  }
  
  bool RCSotController::
  readParams(ros::NodeHandle &robot_nh)
  {

    /// Read the level of verbosity for the controller (0: quiet, 1: info, 2: debug).
    /// Default to quiet
    readParamsVerbosityLevel(robot_nh);
    
    /// Reads the SoT dynamic library name.
    if (!readParamsSotLibName(robot_nh))
      return false;

    /// Read /sot_controller/simulation_mode to know if we are in simulation mode
    // Defines if we are in simulation node.
    if (robot_nh.hasParam("/sot_controller/simulation_mode")) 
      simulation_mode_ = true;
    
    /// Read URDF file.
    readUrdf(robot_nh);
    
    /// Calls readParamsJointNames
    // Reads the list of joints to be controlled.
    if (!readParamsJointNames(robot_nh))
      return false;

    /// Calls readParamsControlMode.
    // Defines if the control mode is position or effort
    if (!readParamsControlMode(robot_nh))
      return false;

    /// Calls readParamsFromRCToSotDevice
    // Mapping from ros-controll to sot device
    readParamsFromRCToSotDevice(robot_nh);

    /// Get control period
    if (!readParamsdt(robot_nh))
      return false;
    
    readParamsEffortControlPDMotorControlData(robot_nh);
    readParamsVelocityControlPDMotorControlData(robot_nh);
    readParamsPositionControlData(robot_nh);
    return true;
  }

    
  bool RCSotController::
  initJoints()
  {
    // Init Joint Names.
    //    joints_.resize(joints_name_.size());


    for (unsigned int i=0;i<nbDofs_;i++)
      {
	bool notok=true;

	while (notok)
	  {
	    std::string &joint_name = joints_name_[i];
	    try 
	      {
		JointSotHandle &aJointSotHandle = joints_[joint_name];
                switch (aJointSotHandle.ros_control_mode)
                {
                  case POSITION:
		    aJointSotHandle.joint = pos_iface_->getHandle(joint_name);
		    if (verbosity_level_>0)
		      ROS_INFO_STREAM("Found joint " << joint_name << " in position "
				      << i << " " << aJointSotHandle.joint.getName());
                    break;
                  case VELOCITY:
		    aJointSotHandle.joint = vel_iface_->getHandle(joint_name);
		    if (verbosity_level_>0)
		      ROS_INFO_STREAM("Found joint " << joint_name << " in velocity "
				      << i << " " << aJointSotHandle.joint.getName());
                    break;
                  case EFFORT:
                    aJointSotHandle.joint = effort_iface_->getHandle(joint_name);
                    if (verbosity_level_>0)
                      ROS_INFO_STREAM("Found joint " << joint_name << " in effort "
                          << i << " " << aJointSotHandle.joint.getName());
                }

		// throws on failure
		notok=false;
		aJointSotHandle.desired_init_pose =
		  aJointSotHandle.joint.getPosition();

	      }
	    catch (...)
	      {
		ROS_ERROR_STREAM("Could not find joint " 
				 << joint_name);
		return false;
	      }
	  }
      }
        
    return true ;
    
  }
  
  bool RCSotController::
  initIMU()
  {
    if (!imu_iface_) return false;

    // get all imu sensor names
    const std :: vector<std :: string >& imu_iface_names = imu_iface_->getNames();
    if (verbosity_level_>0)
      {
	for (unsigned i=0; i <imu_iface_names.size(); i++)
	  ROS_INFO("Got sensor %s", imu_iface_names[i].c_str());
      }
    for (unsigned i=0; i <imu_iface_names.size(); i++){
      // sensor handle on imu
      imu_sensor_.push_back(imu_iface_->getHandle(imu_iface_names[i]));
    }
 
    return true ;
  }
  
  bool RCSotController::
  initForceSensors()
  {
    if (!ft_iface_) return false;

    // get force torque sensors names package.
    const std::vector<std::string>& ft_iface_names = ft_iface_->getNames();
    if (verbosity_level_>0)
      {
	for (unsigned i=0; i <ft_iface_names.size(); i++)
	  ROS_INFO("Got sensor %s", ft_iface_names[i].c_str());
      }
    for (unsigned i=0; i <ft_iface_names.size(); i++){
      // sensor handle on torque forces
      ft_sensors_.push_back(ft_iface_->getHandle(ft_iface_names[i]));
    }
    profileLog_.nbForceSensors = ft_iface_names.size();
    return true;
  }

  bool RCSotController::
  initTemperatureSensors()
  {
    if (!simulation_mode_)
      {
#ifdef TEMPERATURE_SENSOR_CONTROLLER
        if (!act_temp_iface_) return false;

	// get temperature sensors names
	const std::vector<std::string>& act_temp_iface_names = act_temp_iface_->getNames();
	
	if (verbosity_level_>0)
	  {
	    ROS_INFO("Actuator temperature sensors: %ld",act_temp_iface_names.size() ); 
	    
	    for (unsigned i=0; i <act_temp_iface_names.size(); i++)
	      ROS_INFO("Got sensor %s", act_temp_iface_names[i].c_str());
	  }
	
	for (unsigned i=0; i <act_temp_iface_names.size(); i++){
	  // sensor handle on actuator temperature
	  act_temp_sensors_.push_back(act_temp_iface_->getHandle(act_temp_iface_names[i]));
	}
#endif	
      }

    return true;
  }
  
  void RCSotController::
  fillSensorsIn(std::string &title, std::vector<double> & data)
  {
    /// Tries to find the mapping from the local validation
    /// to the SoT device.
    it_map_rt_to_sot it_mapRC2Sot= mapFromRCToSotDevice_.find(title);
    /// If the mapping is found
    if (it_mapRC2Sot!=mapFromRCToSotDevice_.end())
      {
	/// Expose the data to the SoT device.
	std::string lmapRC2Sot = it_mapRC2Sot->second;
	sensorsIn_[lmapRC2Sot].setName(lmapRC2Sot);
	sensorsIn_[lmapRC2Sot].setValues(data);
      }
  }

  void RCSotController::
  fillJoints()
  {
    
    /// Fill positions, velocities and torques.
    for(unsigned int idJoint=0;
	idJoint<joints_name_.size();
	idJoint++)
      {
	it_joint_sot_h anItJoint = joints_.find(joints_name_[idJoint]);
	if (anItJoint!=joints_.end())
	  {
	    JointSotHandle & aJoint = anItJoint->second;
	    DataOneIter_.motor_angle[idJoint] = aJoint.joint.getPosition();
	    
#ifdef TEMPERATURE_SENSOR_CONTROLLER
	    DataOneIter_.joint_angle[idJoint] = aJoint.joint.getAbsolutePosition();
#endif	  
	    DataOneIter_.velocities[idJoint] = aJoint.joint.getVelocity();

#ifdef TEMPERATURE_SENSOR_CONTROLLER	
	    DataOneIter_.torques[idJoint] = aJoint.joint.getTorqueSensor();
#endif
	    DataOneIter_.motor_currents[idJoint] = aJoint.joint.getEffort();
	  }
      }
    
    /// Update SoT internal values
    std::string ltitle("motor-angles"); 
    fillSensorsIn(ltitle,DataOneIter_.motor_angle);
    ltitle = "joint-angles";
    fillSensorsIn(ltitle,DataOneIter_.joint_angle);
    ltitle = "velocities";
    fillSensorsIn(ltitle,DataOneIter_.velocities);
    ltitle = "torques";
    fillSensorsIn(ltitle,DataOneIter_.torques);
    ltitle = "currents";
    fillSensorsIn(ltitle,DataOneIter_.motor_currents);
    
  }

  void RCSotController::setSensorsImu(std::string &name,
					   int IMUnb,
					   std::vector<double> & data)
  {
    std::ostringstream labelOss;
    labelOss << name << IMUnb;
    std::string label_s = labelOss.str();
    fillSensorsIn(label_s,data);
  }

  void RCSotController::
  fillImu()
  {
    for(unsigned int idIMU=0;idIMU<imu_sensor_.size();idIMU++)
      {
	/// Fill orientations, gyrometer and acceleration from IMU.
	if (imu_sensor_[idIMU].getOrientation())
	  {
	    for(unsigned int idquat = 0;idquat<4;idquat++)
	      {
		DataOneIter_.orientation[idquat] = imu_sensor_[idIMU].getOrientation ()[idquat];
	      }
	  }
	if (imu_sensor_[idIMU].getAngularVelocity())
	  {
	    for(unsigned int idgyrometer = 0;idgyrometer<3;
		idgyrometer++)
	      {
		DataOneIter_.gyrometer[idgyrometer] = 
		  imu_sensor_[idIMU].getAngularVelocity()[idgyrometer];
	      }
	  }
	if (imu_sensor_[idIMU].getLinearAcceleration())
	  {
	    for(unsigned int idlinacc = 0;idlinacc<3;
		idlinacc++)
	      {
		DataOneIter_.accelerometer[idlinacc] = 
		  imu_sensor_[idIMU].getLinearAcceleration()[idlinacc];
	      }
	  }
	
	std::string orientation_s("orientation_");
	setSensorsImu(orientation_s, idIMU, DataOneIter_.orientation);

	std::string gyrometer_s("gyrometer_");
	setSensorsImu(gyrometer_s, idIMU, DataOneIter_.gyrometer);

	std::string accelerometer_s("accelerometer_");
	setSensorsImu(accelerometer_s, idIMU, DataOneIter_.accelerometer);
      }
  }
  
  void RCSotController::
  fillForceSensors()
  {
    for(unsigned int idFS=0;idFS<ft_sensors_.size();
	idFS++)
      {
	for(unsigned int idForce=0;idForce<3;idForce++)
	  DataOneIter_.force_sensors[idFS*6+idForce]=
	    ft_sensors_[idFS].getForce()[idForce];
	for(unsigned int idTorque=0;idTorque<3;idTorque++)
	  DataOneIter_.force_sensors[idFS*6+3+idTorque]=
	    ft_sensors_[idFS].getTorque()[idTorque];
      }

    
    std::string alabel("forces");
    fillSensorsIn(alabel,DataOneIter_.force_sensors);
  }

  void RCSotController::
  fillTempSensors()
  {
    if (!simulation_mode_)
      {
#ifdef TEMPERATURE_SENSOR_CONTROLLER
	for(unsigned int idFS=0;idFS<act_temp_sensors_.size();idFS++)
	  {
	    DataOneIter_.temperatures[idFS]=  act_temp_sensors_[idFS].getValue();
	  }
#endif
      }
    else
      {
	for(unsigned int idFS=0;idFS<nbDofs_;idFS++)
	  DataOneIter_.temperatures[idFS]=  0.0;
      }

    std::string alabel("act-temp");
    fillSensorsIn(alabel,DataOneIter_.temperatures);
  }

  void RCSotController::
  fillSensors()
  {
    fillJoints();
    fillImu();
    fillForceSensors();
    fillTempSensors();
  }
  
  void RCSotController::
  readControl(std::map<std::string,dgs::ControlValues> &controlValues)
  {
    ODEBUG4("joints_.size() = " << joints_.size());
    std::string cmdTitle="control";

    it_map_rt_to_sot it_mapRC2Sot= mapFromRCToSotDevice_.find(cmdTitle);
    if (it_mapRC2Sot!=mapFromRCToSotDevice_.end())
      {
	std::string &lmapRC2Sot = it_mapRC2Sot->second;
	command_ = controlValues[lmapRC2Sot].getValues();
	ODEBUG4("angleControl_.size() = " << command_.size());
	for(unsigned int i=0;
	    i<command_.size();++i)
	  {
	    joints_[joints_name_[i]].joint.setCommand(command_[i]);
            DataOneIter_.controls[i] = command_[i];
          }

      }
    else
      ROS_INFO_STREAM("no control.");
  }

  void RCSotController::one_iteration()
  {
    // Chrono start
    RcSotLog_.start_it();
    
    /// Update the sensors.
    fillSensors();

    /// Generate a control law.
    try
      {
	sotController_->nominalSetSensors(sensorsIn_);
	sotController_->getControl(controlValues_);
      }
    catch(std::exception &e) { throw e;}

    /// Read the control values
    readControl(controlValues_);
    
    // Chrono stop.
    RcSotLog_.stop_it();
    
    /// Store everything in Log.
    RcSotLog_.record(DataOneIter_);
  }

  void RCSotController::
  localStandbyEffortControlMode(const ros::Duration& period)
  {
    // ROS_INFO("Compute command for effort mode: %d %d",joints_.size(),effort_mode_pd_motors_.size());
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
	std::string joint_name = joints_name_[idJoint];
	std::map<std::string,ControlPDMotorControlData>::iterator
	  search_ecpd = effort_mode_pd_motors_.find(joint_name);
             
	if (search_ecpd!=effort_mode_pd_motors_.end())
	  {
	    ControlPDMotorControlData & ecpdcdata = search_ecpd->second;
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::JointHandle &aJoint = aJointSotHandle.joint;

	    double vel_err = 0 - aJoint.getVelocity();
	    double err = aJointSotHandle.desired_init_pose - aJoint.getPosition();
	    
	    double local_command = ecpdcdata.pid_controller.computeCommand(err,vel_err,period);
	    // Apply command
	    aJoint.setCommand(local_command);
	  }
      }
  }

  void RCSotController::
  localStandbyVelocityControlMode(const ros::Duration& period)
  {
    static bool first_time=true;
    
    /// Iterate over all the joints
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
        /// Find the joint
        std::string joint_name = joints_name_[idJoint];
	std::map<std::string,ControlPDMotorControlData>::iterator
	  search_ecpd = velocity_mode_pd_motors_.find(joint_name);
             
	if (search_ecpd!=velocity_mode_pd_motors_.end())
	  {
	    ControlPDMotorControlData & ecpdcdata = search_ecpd->second;
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::JointHandle &aJoint = aJointSotHandle.joint;

	    double vel_err = 0 - aJoint.getVelocity();
	    double err = aJointSotHandle.desired_init_pose - aJoint.getPosition();
	    
	    double local_command = ecpdcdata.pid_controller.computeCommand(err,vel_err,period);
		    
	    aJoint.setCommand(local_command);
	    
	    assert(aJoint.getName() == joint_name);
	    if (first_time)
	      if (verbosity_level_>1) {
		ROS_INFO("Control joint %s (id %d) to %f\n",
			 joint_name.c_str(),idJoint,
			 aJoint.getPosition());
	      }
	  }
      }
    first_time=false;
  }
  
  void RCSotController::
  localStandbyPositionControlMode()
  {
    static bool first_time=true;
    
    /// Iterate over all the joints
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
        /// Find the joint
        std::string joint_name = joints_name_[idJoint];
	
	// If it is position mode control.
	if (joints_[joint_name].ros_control_mode==POSITION)
	  {	    
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::JointHandle &aJoint = aJointSotHandle.joint;
		    
	    aJoint.setCommand(aJointSotHandle.desired_init_pose);
	    
	    assert(aJoint.getName() == joint_name);
	    if (first_time)
	      if (verbosity_level_>1) {
		ROS_INFO("Control joint %s (id %d) to %f\n",
			 joint_name.c_str(),idJoint,
			 aJoint.getPosition());
	      }
	  }
      }
    first_time=false;
  }
  
  void RCSotController::
  update(const ros::Time&, const ros::Duration& period)
   {
     // Do not send any control if the dynamic graph is not started
     if (!isDynamicGraphStopped())
       {
	 try
	   {
	     double periodInSec = period.toSec();
	     if (periodInSec+accumulated_time_>dt_-jitter_)
	       {
		 one_iteration();
		 accumulated_time_ = 0.0;
	       }
	     else
	       accumulated_time_ += periodInSec;
	   }
	 catch (std::exception const &exc)
	   {
	     std::cerr << "Failure happened during one_iteration evaluation: std_exception" << std::endl;
	     std::cerr << "Use gdb on this line together with gdb to investiguate the problem: " <<std::endl;
	     std::cerr << __FILE__ << " " << __LINE__  << std::endl;
	     throw exc;
	   }
	 catch (...)
	   {
	     std::cerr << "Failure happened during one_iteration evaluation: unknown exception" << std::endl;
	     std::cerr << "Use gdb on this line together with gdb to investiguate the problem: " <<std::endl;
	     std::cerr << __FILE__ << " " << __LINE__  << std::endl;
	   }
       }
     else
       {
	 // But in effort mode it means that we are sending 0
	 // Therefore implements a default PD controller on the system.
	 // Applying both to handle mixed system.
	 localStandbyEffortControlMode(period);
         localStandbyVelocityControlMode(period);
	 localStandbyPositionControlMode();
       }
   }
  
  void RCSotController::
  starting(const ros::Time &)
  {
    using namespace ::dynamicgraph;
    RealTimeLogger::instance().addOutputStream(LoggerStreamPtr_t(new LoggerROSStream()));

    fillSensors();
  }
    
  void RCSotController::
  stopping(const ros::Time &)
  {
    std::string afilename("/tmp/sot.log");
    RcSotLog_.save(afilename);

    SotLoaderBasic::CleanUp();

    using namespace ::dynamicgraph;
    RealTimeLogger::destroy();
  }
  

  PLUGINLIB_EXPORT_CLASS(sot_controller::RCSotController, 
			 lci::ControllerBase)
}
