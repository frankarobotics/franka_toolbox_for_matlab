//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#ifndef CONTROL_MODES_H
#define CONTROL_MODES_H

#include <string.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "robot_api.h"

#include <franka/exception.h>
#include <franka/robot.h>

void controlTorque(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlTorqueJointPositions(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlTorqueJointVelocities(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlTorqueCartesianPose(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlTorqueCartesianVelocities(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlJointPositionsControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlJointVelocitiesControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlCartesianPoseControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlCartesianVelocitiesControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlTorqueCartesianVelocitiesWithElbow(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlTorqueCartesianPoseWithElbow(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlCartesianPoseControllerModeWithElbow(SimulinkFrankaRobot& simulinkFrankaRobot);
void controlCartesianVelocitiesControllerModeWithElbow(SimulinkFrankaRobot& simulinkFrankaRobot);

extern void (*controlModeCallbackFunction[13])(SimulinkFrankaRobot& simulinkFrankaRobot);

#endif