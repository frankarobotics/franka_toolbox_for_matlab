//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#ifndef ROBOT_API_H
#define ROBOT_API_H

#include <string.h>
#include <vector>

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "franka_matlab_utils.h"

#include "structs.h"
       
class SimulinkFrankaRobot{
    
    // Mask Parameters

    std::string robotIPString;
    
    int gripperAttached;
    int controlMode;
    int initialJointConfigurationRequested;
    int elbowControl;
    int rateLimiter;
    
    std::array<double, 52> collisionThresholds;
    std::array<double, 7> jointImpedance;
    std::array<double, 6> cartesianImpedance;
    std::array<double, 13> loadInertia;
    std::array<double, 16> NE_T_EE;
    std::array<double, 16> EE_T_K;
    std::array<double, 7> initialJointConfiguration;

    franka::RobotState robotState;
    std::unique_ptr<franka::Model> robotModel;
            
    franka::ControllerMode controllerMode;
    
    bool currentlyInFirstControlStep;
    bool computationalPriorityIsFigured;
    
    bool terminateControlThread;
    bool controlThreadException;
    
    double cutoffFrequency;
    double sampleTime;
         
    struct ControllerInputsStruct controllerInputs;
    
    struct ControlModeMembersBegin controlModeMembersBegin[13];
    
    std::thread controlThread;
    
    bool controlThreadHasBeenSpawned;
    
    std::mutex mutexForMainControlStepCycle;
    std::mutex mutexForEnsuringCurrentStepStepControlSignal;
    
    std::condition_variable conditionVariableForMainControlStepCycle;
    std::condition_variable conditionVariableForEnsuringCurrentStepStepControlSignal;
    
    bool controlThreadStepDone;
    bool inputsHaveBeenCopied;
    
public:
    
    std::unique_ptr<franka::Robot> robot;
    
    /**
     * @brief Default constructor for SimulinkFrankaRobot.
     */
    SimulinkFrankaRobot();
    
    /**
     * @brief Constructs a SimulinkFrankaRobot with specified settings.
     *
     * @param robotIPmask IP address of the robot.
     * @param gripperAttachedMask Indicates if the gripper is attached.
     * @param controlModeMask Control mode setting.
     * @param controllerChoiceMask Controller choice setting.
     * @param collisionThresholdsMask Collision thresholds.
     * @param jointImpedanceMask Joint impedance settings.
     * @param cartesianImpedanceMask Cartesian impedance settings.
     * @param loadInertiaMask Load inertia settings.
     * @param NE_T_EEMask Transformation matrix NE_T_EE.
     * @param EE_T_KMask Transformation matrix EE_T_K.
     * @param initialJointConfigurationRequestedMask Initial joint configuration request.
     * @param initialJointConfigurationMask Initial joint configuration.
     * @param elbowControlMask Elbow control setting.
     * @param rateLimiterMask Rate limiter setting.
     * @param cutoffFrequencyMask Cutoff frequency setting.
     */
    SimulinkFrankaRobot(const char * robotIPmask,
                       const int gripperAttachedMask,
                       const int controlModeMask,
                       const int controllerChoiceMask,
                       const double *collisionThresholdsMask,
                       const double *jointImpedanceMask,
                       const double *cartesianImpedanceMask,
                       const double *loadInertiaMask,
                       const double *NE_T_EEMask,
                       const double *EE_T_KMask, 
                       const int initialJointConfigurationRequestedMask,
                       const double *initialJointConfigurationMask,
                       const int elbowControlMask,
                       const int rateLimiterMask,
                       const double cutoffFrequencyMask);
    
    /**
     * @brief Move assignment operator.
     *
     * @param otherSimulinkFrankaRobot Another SimulinkFrankaRobot object.
     * @return Reference to this object.
     */
    SimulinkFrankaRobot& operator=(SimulinkFrankaRobot&& otherSimulinkFrankaRobot);
    
    /**
     * @brief Destructor for SimulinkFrankaRobot.
     */
    ~SimulinkFrankaRobot();
    
    /**
     * @brief Parses mask settings for the robot.
     *
     * @param robotIPmask IP address of the robot.
     * @param gripperAttachedMask Indicates if the gripper is attached.
     * @param controlModeMask Control mode setting.
     * @param controllerChoiceMask Controller choice setting.
     * @param collisionThresholdsMask Collision thresholds.
     * @param jointImpedanceMask Joint impedance settings.
     * @param cartesianImpedanceMask Cartesian impedance settings.
     * @param loadInertiaMask Load inertia settings.
     * @param NE_T_EEMask Transformation matrix NE_T_EE.
     * @param EE_T_KMask Transformation matrix EE_T_K.
     * @param initialJointConfigurationRequestedMask Initial joint configuration request.
     * @param initialJointConfigurationMask Initial joint configuration.
     * @param elbowControlMask Elbow control setting.
     * @param rateLimiterMask Rate limiter setting.
     * @param cutoffFrequencyMask Cutoff frequency setting.
     */
    void parseMaskSettings(const char * robotIPmask,
                           const int gripperAttachedMask,
                           const int controlModeMask,
                           const int controllerChoiceMask,
                           const double *collisionThresholdsMask,
                           const double *jointImpedanceMask,
                           const double *cartesianImpedanceMask,
                           const double *loadInertiaMask,
                           const double *NE_T_EEMask,
                           const double *EE_T_KMask, 
                           const int initialJointConfigurationRequestedMask,
                           const double *initialJointConfigurationMask,
                           const int elbowControlMask,
                           const int rateLimiterMask,
                           const double cutoffFrequencyMask);
    
    /**
     * @brief Applies collision behavior settings to the robot.
     */
    void applyCollisionBehaviorSettings();
    
    /**
     * @brief Applies robot settings.
     */
    void applyRobotSettings();
    
    /**
     * @brief Establishes if the current block is the first to be computed.
     *
     * @return Integer indicating the result.
     */
    int establishIfCurrentBlockFirstToBeComputed();
    
    /**
     * @brief Checks for and handles exceptions.
     */
    void checkIfAndHandleException();
    
    /**
     * @brief Sets control mode members at the beginning.
     */
    void setControlModeMembersBegin();
    
    /**
     * @brief Gets the robot state signal pointer for a given string literal.
     *
     * @param signalNameString The name of the signal.
     * @return Pointer to the robot state signal.
     */
    const double* getRobotStateSignalPointerForStringLiteral(const std::string& signalNameString);
    
    /**
     * @brief Copies an input signal.
     *
     * @param inputSignal The input signal to copy.
     * @param inputSignalIndex The index of the input signal.
     */
    void copyInputSignal(const double *inputSignal, const int inputSignalIndex);
    
    /**
     * @brief Copies an output signal.
     *
     * @param outputSignalsStack Stack of output signals.
     * @param outputSignalIndex The index of the output signal.
     * @param outputSignal The output signal to copy.
     */
    void copyOutputSignal(std::vector<std::string>& outputSignalsStack, int outputSignalIndex, double *outputSignal);
    
    /**
     * @brief Spawns the control thread.
     */
    void spawnControlThread();
    
    /**
     * @brief Notifies the control thread to continue.
     */
    void notifyControlThreadToContinue();
    
    /**
     * @brief Waits for the control thread step to complete.
     */
    void waitForControlThreadStep();
    
    /**
     * @brief Notifies the main thread to continue.
     */
    void notifyMainThreadToContinue();
    
    /**
     * @brief Waits for notification from the main thread to continue.
     */
    void waitNotificationFromMainThreadToContinue();
    
    /**
     * @brief Mounts the robot state.
     *
     * @param currentRobotState The current robot state.
     */
    void mountRobotState(const franka::RobotState& currentRobotState);
    
    /**
     * @brief Gets the current robot state.
     *
     * @return The current robot state.
     */
    const franka::RobotState getRobotState();
    
    /**
     * @brief Checks if the control thread has been spawned.
     *
     * @return True if the control thread has been spawned, false otherwise.
     */
    bool getControlThreadHasBeenSpawned() const;
    
    /**
     * @brief Checks if currently in the first control step.
     *
     * @return True if in the first control step, false otherwise.
     */
    bool getCurrentlyInFirstControlStep() const;
    
    /**
     * @brief Sets the sample time.
     *
     * @param currentSampleTime The current sample time.
     */
    void setSampleTime(double currentSampleTime);
    
    /**
     * @brief Gets the sample time.
     *
     * @return The sample time.
     */
    double getSampleTime() const;
    
    /**
     * @brief Gets the elbow control setting.
     *
     * @return The elbow control setting.
     */
    int getElbowControl() const;
    
    /**
     * @brief Copies the robot pose.
     *
     * @param frame The frame of reference.
     * @param outputSignal The output signal to copy the pose to.
     */
    void copyRobotPose(int frame, double *outputSignal);
    
    /**
     * @brief Copies the robot pose with additional parameters.
     *
     * @param frame The frame of reference.
     * @param q Joint positions.
     * @param F_T_EE Transformation matrix F_T_EE.
     * @param EE_T_K Transformation matrix EE_T_K.
     * @param outputSignal The output signal to copy the pose to.
     */
    void copyRobotPose(int frame,
                       const double *q,
                       const double *F_T_EE,
                       const double *EE_T_K,
                       double *outputSignal);
    
    /**
     * @brief Copies the Jacobian.
     *
     * @param type The type of Jacobian.
     * @param frame The frame of reference.
     * @param outputSignal The output signal to copy the Jacobian to.
     */
    void copyJacobian(int type, 
                      int frame, 
                      double *outputSignal);
    
    /**
     * @brief Copies the Jacobian with additional parameters.
     *
     * @param type The type of Jacobian.
     * @param frame The frame of reference.
     * @param q Joint positions.
     * @param F_T_EE Transformation matrix F_T_EE.
     * @param EE_T_K Transformation matrix EE_T_K.
     * @param outputSignal The output signal to copy the Jacobian to.
     */
    void copyJacobian(int type, 
                      int frame,                        
                      const double *q,
                      const double *F_T_EE,
                      const double *EE_T_K, 
                      double *outputSignal);
    
    /**
     * @brief Copies the mass matrix.
     *
     * @param outputSignal The output signal to copy the mass matrix to.
     */
    void copyMass(double *outputSignal);
    
    /**
     * @brief Copies the mass matrix with additional parameters.
     *
     * @param q Joint positions.
     * @param I_total Total inertia.
     * @param m_total Total mass.
     * @param F_x_Ctotal Center of mass.
     * @param outputSignal The output signal to copy the mass matrix to.
     */
    void copyMass(const double *q,
                  const double *I_total,
                  const double *m_total,
                  const double *F_x_Ctotal,
                  double *outputSignal);
    
    /**
     * @brief Copies the Coriolis forces.
     *
     * @param outputSignal The output signal to copy the Coriolis forces to.
     */
    void copyCoriolis(double *outputSignal);
    
    /**
     * @brief Copies the Coriolis forces with additional parameters.
     *
     * @param q Joint positions.
     * @param dq Joint velocities.
     * @param I_total Total inertia.
     * @param m_total Total mass.
     * @param F_x_Ctotal Center of mass.
     * @param outputSignal The output signal to copy the Coriolis forces to.
     */
    void copyCoriolis(const double *q,
                      const double *dq,
                      const double *I_total,
                      const double *m_total,
                      const double *F_x_Ctotal,
                      double *outputSignal);
    
    /**
     * @brief Copies the gravity vector.
     *
     * @param outputSignal The output signal to copy the gravity vector to.
     */
    void copyGravity(double *outputSignal);
    
    /**
     * @brief Copies the gravity vector with additional parameters.
     *
     * @param gravity_earth Gravity vector on Earth.
     * @param outputSignal The output signal to copy the gravity vector to.
     */
    void copyGravity(const double *gravity_earth,
                     double *outputSignal);
    
    /**
     * @brief Copies the gravity vector with additional parameters.
     *
     * @param q Joint positions.
     * @param m_total Total mass.
     * @param F_x_Ctotal Center of mass.
     * @param outputSignal The output signal to copy the gravity vector to.
     */
    void copyGravity(const double *q,
                     const double *m_total,
                     const double *F_x_Ctotal, 
                     double *outputSignal);
    
    /**
     * @brief Copies the gravity vector with additional parameters.
     *
     * @param q Joint positions.
     * @param m_total Total mass.
     * @param F_x_Ctotal Center of mass.
     * @param gravity_earth Gravity vector on Earth.
     * @param outputSignal The output signal to copy the gravity vector to.
     */
    void copyGravity(const double *q,
                     const double *m_total,
                     const double *F_x_Ctotal,
                     const double *gravity_earth, 
                     double *outputSignal);
    
    /**
     * @brief Sets the flag for the first control step.
     *
     * @param controlStepFlag The flag indicating the first control step.
     */
    void setCurrentlyInFirstControlStep(bool controlStepFlag);
    
    /**
     * @brief Sets the control thread exception flag.
     */
    void setControlThreadExceptionFlag();
    
    /**
     * @brief Checks if the control thread should be terminated.
     *
     * @return True if the control thread should be terminated, false otherwise.
     */
    bool getTerminateControlThread() const;
    
    /**
     * @brief Gets the controller mode.
     *
     * @return The controller mode.
     */
    franka::ControllerMode getControllerMode() const;
    
    /**
     * @brief Checks if the rate limiter is enabled.
     *
     * @return True if the rate limiter is enabled, false otherwise.
     */
    bool getRateLimiter() const;
    
    /**
     * @brief Gets the cutoff frequency.
     *
     * @return The cutoff frequency.
     */
    double getCutoffFrequency() const;
    
    /**
     * @brief Gets the controller input torques.
     *
     * @return Array of controller input torques.
     */
    std::array<double, 7> const &getControllerInputTorques() const;
    
    /**
     * @brief Gets the controller input joint positions.
     *
     * @return Array of controller input joint positions.
     */
    std::array<double, 7> const &getControllerInputJointPositions() const;
    
    /**
     * @brief Gets the controller input joint velocities.
     *
     * @return Array of controller input joint velocities.
     */
    std::array<double, 7> const &getControllerInputJointVelocities() const;
    
    /**
     * @brief Gets the controller input Cartesian pose.
     *
     * @return Array of controller input Cartesian pose.
     */
    std::array<double, 16> const &getControllerInputCartesianPose() const;
    
    /**
     * @brief Gets the controller input Cartesian velocities.
     *
     * @return Array of controller input Cartesian velocities.
     */
    std::array<double, 6> const &getControllerInputCartesianVelocities() const;
    
    /**
     * @brief Gets the controller input elbow position.
     *
     * @return Array of controller input elbow position.
     */
    std::array<double, 2> const &getControllerInputElbowPosition() const;
      
};

#endif