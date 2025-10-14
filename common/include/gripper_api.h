//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#ifndef GRIPPER_API_H
#define GRIPPER_API_H

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>

#include <franka/gripper.h>

enum gripperControlModes  { idle_,
                            read_state_,
                            homing_,
                            grasp_,
                            move_,
                            stop_ };
                                
class SimulinkFrankaGripper{
    
    std::string robotIP;
    std::unique_ptr<franka::Gripper> gripper;
    
    franka::GripperState gripperState;

    int stateNewlyReadCounter = 0;
    int stateNewlyReadCounterPrev = 0;
    int gripperGraspFailed = 0;

    double graspWidth = 0;
    double graspSpeed = 0;
    double graspForce = 0;
    double graspEpsilonInner = 0;
    double graspEpsilonOuter = 0;

    double moveWidth = 0;
    double moveSpeed = 0;

    int gripperCurrentControlMode = idle_;

    int triggerGripperCommand = 0;
    int triggerGripperCommandPrev = 0;

    bool terminateGripperThread = false;
    bool gripperCommandApplied = false;
    
    std::thread gripperControlThread;
    std::mutex m;
    std::condition_variable cv;
    
public:

    /**
     * @brief Constructs a SimulinkFrankaGripper with the specified robot IP.
     *
     * @param robotIPMaskMask IP address of the robot.
     */
    SimulinkFrankaGripper(const char * robotIPMaskMask);

    /**
     * @brief Destructor for SimulinkFrankaGripper.
     */
    ~SimulinkFrankaGripper();

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
     * @brief Gets the current width of the gripper.
     *
     * @return The current gripper width.
     */
    double getGripperWidth();

    /**
     * @brief Gets the maximum width of the gripper.
     *
     * @return The maximum gripper width.
     */
    double getGripperMaxWidth();

    /**
     * @brief Checks if the gripper is currently grasping an object.
     *
     * @return True if the gripper is grasping, false otherwise.
     */
    double getGripperIsGrasped();

    /**
     * @brief Checks if the gripper grasp has failed.
     *
     * @return True if the grasp has failed, false otherwise.
     */
    double getGripperGraspFailed();

    /**
     * @brief Gets the current temperature of the gripper.
     *
     * @return The gripper temperature.
     */
    double getGripperTemperature();

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
     * @brief Performs the homing operation for the gripper.
     */
    void performHoming();

    /**
     * @brief Performs a grasp operation with the gripper.
     */
    void performGrasp();

    /**
     * @brief Performs a move operation with the gripper.
     */
    void performMove();

    /**
     * @brief Stops the gripper operation.
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
};

/**
 * @brief Function to control the gripper in a separate thread.
 *
 * @param simulinkFrankaGripper Reference to a SimulinkFrankaGripper object.
 */
extern void gripperControlThread_func(SimulinkFrankaGripper& simulinkFrankaGripper);

#endif