[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Build Status](https://gitlab.laas.fr/stack-of-tasks/roscontrol_sot/badges/master/pipeline.svg)](http://gitlab.laas.fr/stack-of-tasks/roscontrol_sot)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/roscontrol_sot/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/roscontrol_sot/master/coverage/)

# Introduction
This package encapsulates a SoT graph to control a robot in the ros-control framework.
The intent is to make it generic and adapted to any robot through rosparam.
As the Stack-Of-Taks is a whole-body controller it tries to connect to all the available
resources of the robot.

# rosparam

## Namespace
All the parameters regarding the SoT inside ros-control are in the namespace
```
/sot_controller
```

## Setting the SoT dynamic library which contains the robot device.

Let us assume that you want to control a TALOS robot.
You should then write a YAML file than can be named:
```
sot_talos_param.yaml
```

Its SoT device entity is located inside the following dynamic library:
```
/opt/openrobots/lib/libsot-pyrene-controller.so
```
Then inside the file sot_talos_param.yaml
```
  libname: libsot-pyrene-controller.so
```

## Specifying the actuated state vector
To map the joints from the URDF model to the SoT actuated state vector, it is simply done by giving the ordered list of the joints name in the URDF model.
For instance:
```
     joint_names: [ leg_left_1_joint, leg_left_2_joint, leg_left_3_joint, leg_left_4_joint, leg_left_5_joint, leg_left_6_joint,
    leg_right_1_joint, leg_right_2_joint, leg_right_3_joint, leg_right_4_joint, leg_right_5_joint, leg_right_6_joint,
    torso_1_joint,torso_2_joint,head_1_joint, head_2_joint,
    arm_left_1_joint, arm_left_2_joint, arm_left_3_joint, arm_left_4_joint, arm_left_5_joint, arm_left_6_joint, arm_left_7_joint, gripper_left_joint,
    arm_right_1_joint, arm_right_2_joint, arm_right_3_joint, arm_right_4_joint, arm_right_5_joint, arm_right_6_joint, arm_right_7_joint, gripper_right_joint
  ]
```

## Specifying the map between the ros control data and the sot device entity.

This ros-control plugin connect to the robot (or simulated robot) by requesting the available ressources such as:
1. motor-angles: Reading of the motor position
2. joint-angles: Reading of the joint position
3. velocities: Reading/estimation of the joint velocities
4. torques: Reading/estimation of the joint torques
5. cmd-position: Command for the joint positions
6. cmd-effort: Command for the joint torques

For instance the map for a robot which maps __cmd-position__ to __joints__ and __cmd-effort__ to __effort__ in its device will have
have the following lines in its param file:
```
map_rc_to_sot_device: { motor-angles: motor-angles ,
  joint-angles: joint-angles, velocities: velocities,
  torques: torques, cmd-joints: joints, cmd-effort: effort }
```
## Specifying the control mode
Robots with __ros-control__ can be controlled either in position (POSITION) or in torque (EFFORT)
using the __control_mode__ variable:

```
     control_mode: EFFORT
```

# Logging

Logs of the last 5 minutes are written in `/tmp/sot.log-*` in binary format.
Use command `roscontrol-sot-parse-log /tmp/sot.log-duration.. > txtformat` to get the clear text version.
