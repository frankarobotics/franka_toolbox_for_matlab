//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#include <string.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "robot_api.h"
#include "control_modes_utils.h"

#include <franka/exception.h>
#include <franka/robot.h>

template <typename controllerType>
controllerType standardControlLogic(const franka::RobotState& robotState,
                                       franka::Duration period, SimulinkFrankaRobot& simulinkFrankaRobot) {
  // fetch the current sample time
  simulinkFrankaRobot.setSampleTime(period.toSec());
  
  // update robot state
  simulinkFrankaRobot.mountRobotState(robotState);
 
  // first step flag set
  simulinkFrankaRobot.setCurrentlyInFirstControlStep(false);
  
  // notify mdlOutput that we are entering major control step
  simulinkFrankaRobot.notifyMainThreadToContinue();
  
  // wait till mdlOutput has copied the inputs
  simulinkFrankaRobot.waitNotificationFromMainThreadToContinue();

  if (simulinkFrankaRobot.getTerminateControlThread()) {
    return franka::MotionFinished(handlingParseInfo<controllerType>(simulinkFrankaRobot));
  }
  
  return handlingParseInfo<controllerType>(simulinkFrankaRobot);
}

template <typename controllerType>
void applyControlWithTwoObjectives(SimulinkFrankaRobot& simulinkFrankaRobot) {
  simulinkFrankaRobot.robot->control(
      [&](const franka::RobotState& robotState, franka::Duration period) -> franka::Torques {
        if (simulinkFrankaRobot.getTerminateControlThread()) {
          return franka::MotionFinished(handlingParseInfo<franka::Torques>(simulinkFrankaRobot));
        }
        return handlingParseInfo<franka::Torques>(simulinkFrankaRobot);;
      },
      [&](const franka::RobotState& robotState, franka::Duration period) -> controllerType {
        if (simulinkFrankaRobot.getTerminateControlThread()) {
          return franka::MotionFinished(handlingParseInfo<controllerType>(simulinkFrankaRobot));
        }
        return standardControlLogic<controllerType>(robotState, period,simulinkFrankaRobot);
      },simulinkFrankaRobot.getRateLimiter(),simulinkFrankaRobot.getCutoffFrequency());
}

template <typename controllerType>
void applyControl(SimulinkFrankaRobot& simulinkFrankaRobot) {
  simulinkFrankaRobot.robot->control(
      [&](const franka::RobotState& robotState, franka::Duration period) -> controllerType {
        if (simulinkFrankaRobot.getTerminateControlThread()) {
          return franka::MotionFinished(handlingParseInfo<controllerType>(simulinkFrankaRobot));
        }
        return standardControlLogic<controllerType>(robotState, period,simulinkFrankaRobot);
      },
      simulinkFrankaRobot.getControllerMode(),simulinkFrankaRobot.getRateLimiter(),simulinkFrankaRobot.getCutoffFrequency());
}

template <>
void applyControl<franka::Torques>(SimulinkFrankaRobot& simulinkFrankaRobot) {
  simulinkFrankaRobot.robot->control(
      [&](const franka::RobotState& robotState, franka::Duration period) -> franka::Torques {
        return standardControlLogic<franka::Torques>(robotState, period,simulinkFrankaRobot);
      },simulinkFrankaRobot.getRateLimiter(),simulinkFrankaRobot.getCutoffFrequency());
}

void exceptionHandle(const franka::Exception& e, SimulinkFrankaRobot& simulinkFrankaRobot) {
  
  simulinkFrankaRobot.setControlThreadExceptionFlag();
  simulinkFrankaRobot.notifyMainThreadToContinue();
  std::cout << e.what() << std::endl;
}

void standardTryCatchFunc(void (*robotControl)(SimulinkFrankaRobot&), SimulinkFrankaRobot& simulinkFrankaRobot) {
  try {
    robotControl(simulinkFrankaRobot);
  } catch (const franka::Exception& e) {
    exceptionHandle(e,simulinkFrankaRobot);
  }
}

void controlTorque(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControl<franka::Torques>, simulinkFrankaRobot);
}

void controlTorqueJointPositions(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControlWithTwoObjectives<franka::JointPositions>, simulinkFrankaRobot);
}

void controlTorqueJointVelocities(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControlWithTwoObjectives<franka::JointVelocities>, simulinkFrankaRobot);
}

void controlTorqueCartesianPose(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControlWithTwoObjectives<franka::CartesianPose>, simulinkFrankaRobot);
}

void controlTorqueCartesianVelocities(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControlWithTwoObjectives<franka::CartesianVelocities>, simulinkFrankaRobot);
}

void controlJointPositionsControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControl<franka::JointPositions>, simulinkFrankaRobot);
}

void controlJointVelocitiesControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControl<franka::JointVelocities>, simulinkFrankaRobot);
}

void controlCartesianPoseControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControl<franka::CartesianPose>, simulinkFrankaRobot);
}

void controlCartesianVelocitiesControllerMode(SimulinkFrankaRobot& simulinkFrankaRobot) {
  standardTryCatchFunc(applyControl<franka::CartesianVelocities>, simulinkFrankaRobot);
}

void (*controlModeCallbackFunction[13])(SimulinkFrankaRobot& simulinkFrankaRobot) = { controlTorque,
                                                                                    controlTorqueJointPositions,
                                                                                    controlTorqueJointVelocities,
                                                                                    controlTorqueCartesianPose,
                                                                                    controlTorqueCartesianVelocities,
                                                                                    controlJointPositionsControllerMode,
                                                                                    controlJointVelocitiesControllerMode,
                                                                                    controlCartesianPoseControllerMode,
                                                                                    controlCartesianVelocitiesControllerMode,
                                                                                    controlTorqueCartesianVelocities,
                                                                                    controlTorqueCartesianPose,
                                                                                    controlCartesianPoseControllerMode,
                                                                                    controlCartesianVelocitiesControllerMode};
