//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#ifndef CONTROL_MODES_UTILS_H
#define CONTROL_MODES_UTILS_H

#include <string.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <franka/exception.h>
#include <franka/robot.h>

template<typename T>
T handlingParseInfo(SimulinkFrankaRobot& simulinkFrankaRobot);

template<>
franka::Torques handlingParseInfo<franka::Torques>(SimulinkFrankaRobot& simulinkFrankaRobot){
    return simulinkFrankaRobot.getControllerInputTorques();
}

template<>
franka::JointPositions handlingParseInfo<franka::JointPositions>(SimulinkFrankaRobot& simulinkFrankaRobot){
    return simulinkFrankaRobot.getControllerInputJointPositions();
}

template<>
franka::JointVelocities handlingParseInfo<franka::JointVelocities>(SimulinkFrankaRobot& simulinkFrankaRobot){
    return simulinkFrankaRobot.getControllerInputJointVelocities();
}

template<>
franka::CartesianPose handlingParseInfo<franka::CartesianPose>(SimulinkFrankaRobot& simulinkFrankaRobot){
    if (simulinkFrankaRobot.getElbowControl()){
        return {simulinkFrankaRobot.getControllerInputCartesianPose(), simulinkFrankaRobot.getControllerInputElbowPosition()};
    }else{
        return simulinkFrankaRobot.getControllerInputCartesianPose();
    }
}

template<>
franka::CartesianVelocities handlingParseInfo<franka::CartesianVelocities>(SimulinkFrankaRobot& simulinkFrankaRobot){
    if (simulinkFrankaRobot.getElbowControl()){
        return {simulinkFrankaRobot.getControllerInputCartesianVelocities(), simulinkFrankaRobot.getControllerInputElbowPosition()};
    }else{
        return simulinkFrankaRobot.getControllerInputCartesianVelocities();
    }
}

template<typename T>
constexpr long unsigned int secondaryCallbackInputWidth();

template<>
constexpr long unsigned int secondaryCallbackInputWidth<franka::JointPositions>(){
    return 7;
}

template<>
constexpr long unsigned int secondaryCallbackInputWidth<franka::JointVelocities>(){
    return  7;
}

template<>
constexpr long unsigned int secondaryCallbackInputWidth<franka::CartesianPose>(){
    return 16;
}

template<>
constexpr long unsigned int secondaryCallbackInputWidth<franka::CartesianVelocities>(){
    return 6;
}

#endif