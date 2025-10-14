//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package

#ifndef FRANKA_SIMULINK_LIBRARY_STRUCTS_H
#define FRANKA_SIMULINK_LIBRARY_STRUCTS_H

#include <string.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
#include <array>

struct ControllerInputsStruct{
    // Desired Cartesian Pose O_T_EE_d
    std::array<double, 16> O_T_EE_d;

    // Desired Cartesian Velocity O_dP_EE_d
    std::array<double, 6> O_dP_EE_d;

    // Desired Joint Positions q_d
    std::array<double, 7> q_d;

    // Desired Joint Velocities dq_d
    std::array<double, 7> dq_d;

    // Desired Torque tau_J_d
    std::array<double, 7> tau_J_d;
    
    // Desired Elbow elbow_d
    std::array<double, 2> elbow_d;
};

struct ControlModeSizes{
    int numInputs;
    int width[3];
};

struct ControlModeMembersBegin{
    double *memberBegin[3];
};

#endif