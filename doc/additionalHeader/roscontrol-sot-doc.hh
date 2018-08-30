/* Copyright 2018, CNRS
 * Author: O. Stasse
 * 
BSD 2-Clause License

Copyright (c) 2017, Stack Of Tasks development team
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

/**
\mainpage
\section sec_intro Introduction
This package provides a link between the Stack-Of-Tasks framework and roscontrol.
It assumes that a robot has a 
<a href="https://github.com/ros-controls/ros_control/search?q=hardware_interface&unscoped_q=hardware_interface">hardware_interface</a>  provided by roscontrol.
It also assumes that the robot has a Device class specialized for the robot.
For instance in the passive walker yoyoman available <a href="https://github.com/Gepetto/yoyoman01_robot">here</a>, this device is
provided in the directory sot-yoyoman01.

In general a wrapper is needed to make this Device class usable in the robot itself, or in a simulator (Gazebo, or OpenHRP).
When an abstract interface is provided, which is the case with ros-control, a yaml file is sufficient to do the wrapping.
For two working examples you can have a look to the <a href="https://github.com/stack-of-tasks/talos_metapkg_ros_control_sot">package for the Talos robot</a>,
or the directory 
<a href="https://github.com/Gepetto/yoyoman01_robot/tree/master/yoyoman_metapkg_ros_control_sot">yoyoman_metapkg_ros_control_sot</a> 
for the yoyoman passive walker.

\section sec_install Install

\subsection subsec_install_bin Binary installation

For binary installation we rely on the robotpkg project which is providing an apt repository.
Please follow the instructions given <a href="http://robotpkg.openrobots.org/debian.html">here</a> to access this apt repository.
Once this is done, installing roscontrol_sot can be done with:
\code{.sh}
sudo apt-get install robotpkg-roscontrol-sot
\endcode

WARNING: the roscontrol-sot provided by robotpkg is compiled over the PAL-robotics fork of roscontrol
and not the ros packages roscontrol.

\subsection subsec_install_src Source Installation

First you must install dynamic-graph-bridge-v3:
\code{.sh}
sudo apt-get install robotpkg-dynamic-graph-bridge-v3
\endcode
This should be done after following the previous step to access the robotpkg apt repository.

Then you can clone the repository:
\code{.sh}
git clone https://github.com/stack-of-tasks/roscontrol_sot.git 
\endcode

You can either compile it with:
\code{.sh}
cd roscontrol_sot
make _build
cd _build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/your_path ..
make install
\endcode

or, if you are in a ros workspace:
\code{.sh}
catkin_make
\endcode

\section sec_introduction Overview

\subsection roscontrol

To control a robot it is necessary to implement the following loop
\code{.c}
void rtloop()
{
  ReadSensors();
  UpdateCmd();
  WriteCmd();
}
\endcode

ReadSensors() and the WriteCmd() are
functions accessing the real hardware or a simulator to read the sensors
and write the command respectively.
On the other hand the UpdateCmd() functions implements a the computation of
a control law.
The roscontrol framework is providing an abstract interface to the hardware interface
and the controllers. 

The previous code is then rewritten the following way:
\code{.c}
void * RosControlRealTimeLoop(void *argument)
{
  aRobotHW->read();
  aControllerManager->UpdateCmd();
  aRobotHW->write();
}
\endcode

The interaction between the controller manager and the robot hardware is done
through a hardware interface exposing double which are either publishing sensor
information or control values. 
The abstract interface is allowing to have a normalized way to interact with
the robot hardware.

\subsection subsec_sot Stack-Of-Tasks.

The stack of tasks is loaded thanks to a dynamical library.

In order to exchange informations between the robot and the Stack-Of-Tasks,
maps of doubles are used.
The maps provide sensor information and the value of the control computed by
the SoT.

The size of double vector and the map key are computed automatically depending on the number of sensors and actuators provided
by the robot_hardware_interface.

\section section_yamlfile Setting the YAML file.

\subsection namespace
All the parameters regarding the SoT inside roscontrol are in the namespace
\code{.sh}
/sot_controller
\endcode

\subsection subsection_setsot Setting the SoT dynamic library which contains the robot device.

In order to control a robot, an appropriate YAML file need to be written such as:
\code{.sh}
sot_robot_param.yaml
\endcode

If its SoT device entity is located inside the following dynamic library:
\code{.sh}
/opt/openrobots/lib/libsot-robot-controller.so
\endcode

Then inside the file sot_robot_param.yaml
\code{.yaml}
  libname: libsot-robot-controller.so
\endcode

\subsection subsec_spec_act_state Specifying the actuated state vector
To map the joints from the URDF model to the SoT actuated state vector, it is simply done by giving the ordered list of the joints name in the URDF model.
For instance:
\code{.sh}
     joint_names: [ leg_left_1_joint, leg_left_2_joint, leg_left_3_joint, leg_left_4_joint, leg_left_5_joint, leg_left_6_joint,
    leg_right_1_joint, leg_right_2_joint, leg_right_3_joint, leg_right_4_joint, leg_right_5_joint, leg_right_6_joint,
    torso_1_joint,torso_2_joint,head_1_joint, head_2_joint,
    arm_left_1_joint, arm_left_2_joint, arm_left_3_joint, arm_left_4_joint, arm_left_5_joint, arm_left_6_joint, arm_left_7_joint, gripper_left_joint,
    arm_right_1_joint, arm_right_2_joint, arm_right_3_joint, arm_right_4_joint, arm_right_5_joint, arm_right_6_joint, arm_right_7_joint, gripper_right_joint
  ] 
\endcode

\subsection subsec_spec_map Specifying the map between the ros control data and the sot device entity.

This ros-control plugin connect to the robot (or simulated robot) by requesting the available ressources such as:
<ol>
<li> motor-angles: Reading of the motor position</li>
<li> joint-angles: Reading of the joint position</li>
<li> velocities: Reading/estimation of the joint velocities</li>
<li> torques: Reading/estimation of the joint torques</li>
<li> cmd-position: Command for the joint positions</li>
<li> cmd-effort: Command for the joint torques</li>
</ol>

For instance the map for a robot which maps __cmd-position__ to __joints__ and __cmd-effort__ to __effort__ in its device will have
have the following lines in its param file:
\code{.sh}
map_rc_to_sot_device: { motor-angles: motor-angles ,
  joint-angles: joint-angles, velocities: velocities,
  torques: torques, cmd-joints: joints, cmd-effort: effort }
\endcode

\subsection spec_cont_mode Specifying the control mode
Robots with __ros-control__ can be controlled either in position (POSITION) or in torque (EFFORT)
using the __control_mode__ variable:

\code{.sh}
     control_mode: EFFORT
\endcode

*/
