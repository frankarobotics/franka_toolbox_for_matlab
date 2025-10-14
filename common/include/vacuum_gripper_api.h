//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#ifndef VACUUM_GRIPPER_API_H
#define VACUUM_GRIPPER_API_H

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>

#include <franka/vacuum_gripper.h>

enum vacuumGripperControlModes  { idle_,
                                read_state_,
                                vacuum_,
                                drop_off_,
                                stop_ };
                                
class SimulinkFrankaVacuumGripper{
    
    std::string robotIP;
    std::unique_ptr<franka::VacuumGripper> gripper;
    
    franka::VacuumGripperState vacuumGripperState;

    int stateNewlyReadCounter = 0;
    int stateNewlyReadCounterPrev = 0;

    int vacuumEstablished = 0;

    int gripperCurrentControlMode = idle_;

    int triggerGripperCommand = 0;
    int triggerGripperCommandPrev = 0;

    bool terminateGripperThread = false;
    bool gripperCommandApplied = false;
    
    std::thread gripperControlThread;
    std::mutex m;
    std::condition_variable cv;
    
    uint8_t vacuum_control_point = 0;
    std::chrono::milliseconds vacuum_timeout = std::chrono::milliseconds(0);
    franka::VacuumGripper::ProductionSetupProfile vacuum_profile = franka::VacuumGripper::ProductionSetupProfile::kP0; // 0: kP0, 1: kP1, 2: kP2, 3: kP3

    std::chrono::milliseconds vacuum_timeout_drop_off = std::chrono::milliseconds(0);
    
public:

    /**
     * @brief Constructs a SimulinkFrankaVacuumGripper with the specified robot IP.
     *
     * @param robotIPMaskMask IP address of the robot.
     */
    SimulinkFrankaVacuumGripper(const char * robotIPMaskMask);

    /**
     * @brief Destructor for SimulinkFrankaVacuumGripper.
     */
    ~SimulinkFrankaVacuumGripper();

    /**
     * @brief Checks if a gripper control request has been made.
     *
     * @return True if a control request is made, false otherwise.
     */
    bool gripperControlRequested();

    /**
     * @brief Parses the gripper command from a command vector.
     *
     * @param gripperCommandVector The command vector for the gripper.
     */
    void parseGripperCommand(double *gripperCommandVector);

    /**
     * @brief Applies the parsed gripper command.
     */
    void applyGripperCommand();

    /**
     * @brief Waits for a gripper command to be issued.
     */
    void waitForGripperCommand();

    /**
     * @brief Gets the counter for newly read gripper states.
     *
     * @return The counter for newly read states.
     */
    int getGripperStateNewlyRead();

    /**
     * @brief Checks if the gripper control thread should be terminated.
     *
     * @return True if the thread should be terminated, false otherwise.
     */
    bool getTerminateGripperThread();

    /**
     * @brief Gets the current control mode of the gripper.
     *
     * @return The current gripper control mode.
     */
    int getGripperCurrentControlMode();

    /**
     * @brief Updates the state of the gripper.
     */
    void updateGripperState();

    /**
     * @brief Performs a vacuum operation with the gripper.
     */
    void performVacuum();

    /**
     * @brief Performs a drop off operation with the gripper.
     */
    void performDropOff();

    /**
     * @brief Performs a stop operation with the gripper.
     */
    void performStop();

    /**
     * @brief Equalizes the command counter.
     */
    void equalizeCommandCounter();

    /**
     * @brief Updates the command counter.
     */
    void updateCommandCounter();

    /**
     * @brief Checks if the gripper server is busy.
     *
     * @return True if the server is busy, false otherwise.
     */
    bool gripperServerBusy();

    /**
     * @brief Gets whether the vacuum pressure is in the control range.
     *
     * @return True if pressure is in control range, false otherwise.
     */
    bool getInControlRange();

    /**
     * @brief Gets whether the part is currently detached.
     *
     * @return True if part is detached, false otherwise.
     */
    bool getPartDetached();

    /**
     * @brief Gets whether a part is currently present.
     *
     * @return True if part is present, false otherwise.
     */
    bool getPartPresent();

    /**
     * @brief Gets the current device status.
     *
     * @return The device status as an 8-bit unsigned integer.
     */
    uint8_t getDeviceStatus();

    /**
     * @brief Gets the current actual power consumption.
     *
     * @return The actual power consumption as a 16-bit unsigned integer.
     */
    uint16_t getCurrentActualPower();

    /**
     * @brief Gets the current system vacuum level.
     *
     * @return The system vacuum level as a 16-bit unsigned integer.
     */
    uint16_t getCurrentSystemVacuum();

    /**
     * @brief Gets the timestamp of the current state.
     *
     * @return The timestamp as a franka::Duration object.
     */
    franka::Duration getTime();
};

/**
 * @brief Function to control the vacuum gripper in a separate thread.
 *
 * @param simulinkFrankaVacuumGripper Reference to a SimulinkFrankaVacuumGripper object.
 */
extern void vacuumGripperControlThread_func(SimulinkFrankaVacuumGripper& simulinkFrankaVacuumGripper);

#endif // VACUUM_GRIPPER_API_H 