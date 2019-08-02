/*Sources : 
 * https://github.com/jhu-lcsr-attic/barrett_control/blob/libbarrett/
 barrett_hw/src/wam_server.cpp
 * http://www.ensta-bretagne.fr/lemezo/files/teaching/ROS/TP1.pdf
 * http://homepages.laas.fr/ostasse/Teaching/ROS/rosintro.pdf
 * https://github.com/ros-controls/ros_control/wiki/hardware_interface
 # To compile :g++ ros_hardware.cpp -o ros_harware -I/opt/ros/kinetic/include
 -L/opt/ros/kinetic/lib
*/
#include <iostream>
#include <sstream>

#include <roscontrol_sot_hardware.hh>


//Threads
void *RTloop(void *argument);
void *ROSloop(void *argument);
pthread_mutex_t mutx = PTHREAD_MUTEX_INITIALIZER; //mutex initialisation
//******************************************************


class TestRobot01Class : public hardware_interface::RobotHW
{
public:
  int ReadWrite();

  int UpdateImu();
  int UpdateCmd();
  int UpdateSensor();


  TestRobot01Class() //ros::NodeHandle nh);
  {
    /// connect and register the joint state interface
    hardware_interface::JointStateHandle
      state_handle_base("base", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_base);

    hardware_interface::JointStateHandle
      state_handle_Head("Head", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_Head);

    hardware_interface::JointStateHandle
      state_handle_Neck("Neck", &pos_[2], &vel_[2], &eff_[2]);
    jnt_state_interface_.registerHandle(state_handle_Neck);

    hardware_interface::JointStateHandle
      state_handle_RArm("Rarm", &pos_[3], &vel_[3], &eff_[3]);
    jnt_state_interface_.registerHandle(state_handle_RArm);

    hardware_interface::JointStateHandle
      state_handle_LArm("Larm", &pos_[4], &vel_[4], &eff_[4]);
    jnt_state_interface_.registerHandle(state_handle_LArm);

    hardware_interface::JointStateHandle
      state_handle_RHip("RHip", &pos_[5], &vel_[5], &eff_[5]);
    jnt_state_interface_.registerHandle(state_handle_RHip);

    hardware_interface::JointStateHandle
      state_handle_LHip("LHip", &pos_[6], &vel_[6], &eff_[6]);
    jnt_state_interface_.registerHandle(state_handle_LHip);

    registerInterface(&jnt_state_interface_);

    /// connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_base
      (jnt_state_interface_.getHandle("base"), &cmd_[0]);
    jnt_pos_interface_.registerHandle(pos_handle_base);

    hardware_interface::JointHandle pos_handle_Head
      (jnt_state_interface_.getHandle("Head"), &cmd_[1]);
    jnt_pos_interface_.registerHandle(pos_handle_Head);

    hardware_interface::JointHandle pos_handle_Neck
      (jnt_state_interface_.getHandle("Neck"), &cmd_[2]);
    jnt_pos_interface_.registerHandle(pos_handle_Neck);

    hardware_interface::JointHandle pos_handle_RArm
      (jnt_state_interface_.getHandle("Rarm"), &cmd_[3]);
    jnt_pos_interface_.registerHandle(pos_handle_RArm);

    hardware_interface::JointHandle pos_handle_LArm
      (jnt_state_interface_.getHandle("Larm"), &cmd_[4]);
    jnt_pos_interface_.registerHandle(pos_handle_LArm);

    hardware_interface::JointHandle pos_handle_RHip
      (jnt_state_interface_.getHandle("RHip"), &cmd_[5]);
    jnt_pos_interface_.registerHandle(pos_handle_RHip);

    hardware_interface::JointHandle pos_handle_LHip
      (jnt_state_interface_.getHandle("LHip"), &cmd_[6]);
    jnt_pos_interface_.registerHandle(pos_handle_LHip);

    registerInterface(&jnt_pos_interface_);

    ///IMU
    hardware_interface::ImuSensorHandle::Data anImuData;
    anImuData.name     = "name";
    anImuData.frame_id = "frame_id";
    anImuData.orientation = imu_orientation_;
    anImuData.orientation_covariance =
      imu_orientation_covariance_;
    anImuData.angular_velocity = imu_angular_velocity_;
    anImuData.angular_velocity_covariance =
      imu_angular_velocity_covariance_;
    anImuData.linear_acceleration = imu_linear_acceleration_;
    anImuData.linear_acceleration_covariance =
      imu_linear_acceleration_covariance_;
    
    hardware_interface::ImuSensorHandle
      IMU_handle(anImuData);
    imu_state_interface_.registerHandle(IMU_handle);

    registerInterface(&imu_state_interface_);
  }

  double imu_orientation_[6];
  double imu_orientation_covariance_[9];
  double imu_angular_velocity_[3];
  double imu_angular_velocity_covariance_[9];
  double imu_linear_acceleration_[3];
  double imu_linear_acceleration_covariance_[9];
  
private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::ImuSensorInterface imu_state_interface_;
  double cmd_[7];
  double pos_[7];
  double vel_[7];
  double eff_[7];
};

int TestRobot01Class::ReadWrite()
{
}

int TestRobot01Class::UpdateImu()
{
}

/* get position from STM32 */
int TestRobot01Class::UpdateSensor()
{
  // Write pos
}

/* get cmd from ROS */
int TestRobot01Class::UpdateCmd()
{

  // Read cmd.
}

typedef struct arg_struct
{
  controller_manager::ControllerManager * cm;
  TestRobot01Class *testrobot01;
} RTloopArgs;

void *RTloop(void *argument)
{
  ROS_INFO("IN thread1 OK");
  RTloopArgs * aRTloopArgs;
  aRTloopArgs = (RTloopArgs *)argument;

  ros::Time last_time = ros::Time::now();

  while (ros::ok) /// Boucle tant que le master existe (ros::ok())
    {
      pthread_mutex_lock(&mutx); // On verrouille les variables pour ce thread

      // Read phase
      aRTloopArgs->testrobot01->ReadWrite();
      //Receiving data only one time
      aRTloopArgs->testrobot01->UpdateImu();     //Processing incoming data
      aRTloopArgs->testrobot01->UpdateSensor();  //Processing incoming data

      // Call Controllers
      ros::Duration duration = ros::Time::now() - last_time;
      aRTloopArgs->cm->update(ros::Time::now(), duration);
      last_time = ros::Time::now();

      /// Write Phase
      aRTloopArgs->testrobot01->UpdateCmd();

      pthread_mutex_unlock(&mutx); // On deverrouille les variables
      /// Pause
      usleep(10000); //100HZ
        
    }
  pthread_exit(EXIT_SUCCESS);
}

void *ROSloop(void *argument)
{
  ROS_INFO("IN thread2 OK");
  ros::Rate loop_rate(10); /// Frequence boucle en Hz
  while (ros::ok) // NOT real time loop
    {
      loop_rate.sleep();
    }
  pthread_exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  TestRobot01Class testrobot01;

  /*---------------- ROS Stuff ------------------ */
  /* Initialisation du node : le troisieme argument est son nom */
  ros::init(argc, argv, "hw_node");

  /* Connexion au master et initialisation du NodeHandle qui permet 
     d avoir acces aux topics et services */
  ros::NodeHandle nh;//testrobot01_nh;
  controller_manager::ControllerManager cm(&testrobot01);
  RTloopArgs aRTloopArgs;
  aRTloopArgs.cm = &cm;
  aRTloopArgs.testrobot01 = &testrobot01;

  /*---------------- Initialisation moteurs ------------------ */
  ROS_INFO("Starting in 5s");
  ROS_INFO("-------------------------------- 5");
  sleep(1);
  ROS_INFO("------------------------- 4");
  sleep(1);
  ROS_INFO("----------------- 3");
  sleep(1);
  ROS_INFO("--------- 2");
  sleep(1);
  ROS_INFO("--- 1");
  sleep(1);
    
  /*---------------- Gestion des threads ------------------ */
  /* 
     Thread "number 1" : Main  
     Thread "number 2" : Rosloop
     Thread "number 3" : RTloop
     Thread "number 4" : spinner
  */
  pthread_t thread1;
  pthread_t thread2; 
  int error_return;
    
  struct sched_param params1;
  params1.sched_priority = 90;
  // 1(low) to 99(high)
  error_return = pthread_setschedparam(thread1, SCHED_RR, &params1);
  // function sets the scheduling policy and parameters of the thread
  error_return = pthread_create(&thread1, NULL, RTloop, (void *)
                                &aRTloopArgs);      // create a new thread
  ROS_INFO("thread1 created OK");
  error_return = pthread_create(&thread2, NULL, ROSloop,(void *)
                                &aRTloopArgs);     // create a new thread
  ROS_INFO("thread2 created OK");

  if (error_return)
    {
      ROS_ERROR("return code from pthread_create() is %d\n", error_return);
      exit(-1);
    }

  /* Spinners */
  ros::MultiThreadedSpinner spinner(4); //unspecified (or set to 0),
  // it will use a thread for each CPU core
  spinner.spin();                       // spin() will not return
  // until the node has been shutdown

  return 0;
}
