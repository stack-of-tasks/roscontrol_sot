/*
 * Copyright: LAAS CNRS, 2019
 * Author: O. Stasse
 * LICENSE: See LICENSE.txt
 *
 */
#include <iostream>
#include <sstream>

#include <roscontrol_sot_hardware.hh>

// Threads
void *RTloop(void *argument);

pthread_mutex_t mutx = PTHREAD_MUTEX_INITIALIZER;  // mutex initialisation
//******************************************************

class TestRobot01Class : public hardware_interface::RobotHW {
 public:
  int ReadWrite();

  int UpdateImu();
  int UpdateCmd();
  int UpdateSensor();

  TestRobot01Class()  // ros::NodeHandle nh);
  {
    /// connect and register the joint state interface
    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_1("arm_1_joint", &pos_[0], &vel_[0], &eff_[0]);
#else
        state_handle_arm_1("arm_1_joint", &pos_[0], &vel_[0], &eff_[0],
                           &abs_pos_[0], &abs_torque_[0]);
#endif
    jnt_state_interface_.registerHandle(state_handle_arm_1);

    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_2("arm_2_joint", &pos_[1], &vel_[1], &eff_[1]);
#else
        state_handle_arm_2("arm_2_joint", &pos_[1], &vel_[1], &eff_[1],
                           &abs_pos_[1], &abs_torque_[1]);
#endif

    jnt_state_interface_.registerHandle(state_handle_arm_2);

    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_3("arm_3_joint", &pos_[2], &vel_[2], &eff_[2]);
#else
        state_handle_arm_3("arm_3_joint", &pos_[2], &vel_[2], &eff_[2],
                           &abs_pos_[2], &abs_torque_[2]);
#endif

    jnt_state_interface_.registerHandle(state_handle_arm_3);

    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_4("arm_4_joint", &pos_[3], &vel_[3], &eff_[3]);
#else
        state_handle_arm_4("arm_4_joint", &pos_[3], &vel_[3], &eff_[3],
                           &abs_pos_[3], &abs_torque_[3]);
#endif

    jnt_state_interface_.registerHandle(state_handle_arm_4);

    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_5("arm_5_joint", &pos_[4], &vel_[4], &eff_[4]);
#else
        state_handle_arm_5("arm_5_joint", &pos_[4], &vel_[4], &eff_[4],
                           &abs_pos_[4], &abs_torque_[4]);
#endif

    jnt_state_interface_.registerHandle(state_handle_arm_5);

    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_6("arm_6_joint", &pos_[5], &vel_[5], &eff_[5]);
#else
        state_handle_arm_6("arm_6_joint", &pos_[5], &vel_[5], &eff_[5],
                           &abs_pos_[5], &abs_torque_[5]);
#endif

    jnt_state_interface_.registerHandle(state_handle_arm_6);

    hardware_interface::JointStateHandle
#ifndef TEMPERATURE_SENSOR_CONTROLLER
        state_handle_arm_7("arm_7_joint", &pos_[6], &vel_[6], &eff_[6]);
#else
        state_handle_arm_7("arm_7_joint", &pos_[6], &vel_[6], &eff_[6],
                           &abs_pos_[6], &abs_torque_[6]);
#endif

    jnt_state_interface_.registerHandle(state_handle_arm_7);

    registerInterface(&jnt_state_interface_);

    /// connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_arm_1(
        jnt_state_interface_.getHandle("arm_1_joint"), &cmd_[0]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_1);

    hardware_interface::JointHandle pos_handle_arm_2(
        jnt_state_interface_.getHandle("arm_2_joint"), &cmd_[1]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_2);

    hardware_interface::JointHandle pos_handle_arm_3(
        jnt_state_interface_.getHandle("arm_3_joint"), &cmd_[2]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_3);

    hardware_interface::JointHandle pos_handle_arm_4(
        jnt_state_interface_.getHandle("arm_4_joint"), &cmd_[3]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_4);

    hardware_interface::JointHandle pos_handle_arm_5(
        jnt_state_interface_.getHandle("arm_5_joint"), &cmd_[4]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_5);

    hardware_interface::JointHandle pos_handle_arm_6(
        jnt_state_interface_.getHandle("arm_6_joint"), &cmd_[5]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_6);

    hardware_interface::JointHandle pos_handle_arm_7(
        jnt_state_interface_.getHandle("arm_7_joint"), &cmd_[6]);
    jnt_pos_interface_.registerHandle(pos_handle_arm_7);

    registerInterface(&jnt_pos_interface_);

    /// IMU
    hardware_interface::ImuSensorHandle::Data anImuData;
    anImuData.name = "name";
    anImuData.frame_id = "frame_id";
    anImuData.orientation = imu_orientation_;
    anImuData.orientation_covariance = imu_orientation_covariance_;
    anImuData.angular_velocity = imu_angular_velocity_;
    anImuData.angular_velocity_covariance = imu_angular_velocity_covariance_;
    anImuData.linear_acceleration = imu_linear_acceleration_;
    anImuData.linear_acceleration_covariance =
        imu_linear_acceleration_covariance_;

    hardware_interface::ImuSensorHandle IMU_handle(anImuData);
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
  double abs_pos_[7];
  double abs_torque_[7];
};

int TestRobot01Class::ReadWrite() { return 0; }

int TestRobot01Class::UpdateImu() { return 0; }

/* get position from STM32 */
int TestRobot01Class::UpdateSensor() {
  // Write pos
  return 0;
}

/* get cmd from ROS */
int TestRobot01Class::UpdateCmd() {
  return 0;
  // Read cmd.
}

typedef struct arg_struct {
  controller_manager::ControllerManager *cm;
  TestRobot01Class *testrobot01;
} RTloopArgs;

void *RTloop(void *argument) {
  ROS_INFO("IN thread1 OK");
  RTloopArgs *aRTloopArgs;
  aRTloopArgs = (RTloopArgs *)argument;

  ros::Time last_time = ros::Time::now();

  pthread_t thread1;

  struct sched_param params1;
  params1.sched_priority = 90;
  // 1(low) to 99(high)
  thread1 = pthread_self();
  //  int error_return =
  pthread_setschedparam(thread1, SCHED_RR, &params1);

  unsigned int nb_it = 0;
  while (ros::ok()
         //&& (nb_it < 100)
         )
  /// Boucle tant que le master existe (ros::ok())
  /// and nb_it < 100
  {
    pthread_mutex_lock(&mutx);  // On verrouille les variables pour ce thread

    // Read phase
    aRTloopArgs->testrobot01->ReadWrite();
    // Receiving data only one time
    aRTloopArgs->testrobot01->UpdateImu();     // Processing incoming data
    aRTloopArgs->testrobot01->UpdateSensor();  // Processing incoming data

    // Call Controllers
    ros::Duration duration = ros::Time::now() - last_time;
    aRTloopArgs->cm->update(ros::Time::now(), duration);
    last_time = ros::Time::now();

    /// Write Phase
    aRTloopArgs->testrobot01->UpdateCmd();

    pthread_mutex_unlock(&mutx);  // On deverrouille les variables
    /// Pause
    usleep(10000);  // 100HZ
    nb_it++;
    //      std::cout << "nb_it: " << nb_it << "\n";
  }
  ros::shutdown();
  pthread_exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]) {
  TestRobot01Class testrobot01;

  /*---------------- ROS Stuff ------------------ */
  /* Initialisation du node : le troisieme argument est son nom */
  ros::init(argc, argv, "hw_node");

  /* Connexion au master et initialisation du NodeHandle qui permet
     d avoir acces aux topics et services */
  ros::NodeHandle nh;  // testrobot01_nh;
  controller_manager::ControllerManager cm(&testrobot01);
  RTloopArgs aRTloopArgs;
  aRTloopArgs.cm = &cm;
  aRTloopArgs.testrobot01 = &testrobot01;

  /*---------------- Gestion des threads ------------------ */
  /*
     Thread "number 1" : Main - spinner
     Thread "number 2" : RTloop
  */
  int error_return;
  pthread_t thread1;

  // function sets the scheduling policy and parameters of the thread
  error_return = pthread_create(&thread1, NULL, RTloop,
                                (void *)&aRTloopArgs);  // create a new thread
  ROS_INFO("thread1 created OK");

  if (error_return) {
    ROS_ERROR("return code from pthread_create() is %d\n", error_return);
    exit(-1);
  }

  /* Spinners */
  ros::MultiThreadedSpinner spinner(4);  // unspecified (or set to 0),
  // it will use a thread for each CPU core
  spinner.spin();  // spin() will not return
  // until the node has been shutdown

  return 0;
}
