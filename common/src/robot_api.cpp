//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#include <string.h>
#include <vector>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <map>

#include <franka/exception.h>
#include <franka/robot.h>

#include "robot_api.h"
#include "gripper_api.h"

#include "franka_matlab_robot_utils.h"

SimulinkFrankaRobot::SimulinkFrankaRobot() = default;

SimulinkFrankaRobot::SimulinkFrankaRobot(const char * robotIPmask,
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
                                        const double cutoffFrequencyMask){
    
    parseMaskSettings(robotIPmask,
                      gripperAttachedMask,
                      controlModeMask,
                      controllerChoiceMask,
                      collisionThresholdsMask,
                      jointImpedanceMask,
                      cartesianImpedanceMask,
                      loadInertiaMask,
                      NE_T_EEMask,
                      EE_T_KMask, 
                      initialJointConfigurationRequestedMask,
                      initialJointConfigurationMask,
                      elbowControlMask,
                      rateLimiterMask,
                      cutoffFrequencyMask);
    
    controlThreadHasBeenSpawned = false;    
    controlThreadStepDone = false;
    terminateControlThread = false; 
    controlThreadException = false;
    inputsHaveBeenCopied = false;
    
    bool computationalPriorityIsFigured = false;
    
    currentlyInFirstControlStep = true;
    
    setControlModeMembersBegin();
    
    robot = std::make_unique<franka::Robot>(robotIPString);
    
    robotModel = std::make_unique<franka::Model>(robot->loadModel());
}

SimulinkFrankaRobot& SimulinkFrankaRobot::operator=(SimulinkFrankaRobot&& otherSimulinkFrankaRobot){
    
    if (this == &otherSimulinkFrankaRobot) return *this;
    
    robotIPString = std::move(otherSimulinkFrankaRobot.robotIPString);
    
    gripperAttached = otherSimulinkFrankaRobot.gripperAttached;
    controlMode = otherSimulinkFrankaRobot.controlMode;
    initialJointConfigurationRequested = otherSimulinkFrankaRobot.initialJointConfigurationRequested;
    elbowControl = otherSimulinkFrankaRobot.elbowControl;
    rateLimiter = otherSimulinkFrankaRobot.rateLimiter;
    cutoffFrequency = otherSimulinkFrankaRobot.cutoffFrequency;
    
    collisionThresholds = std::move(otherSimulinkFrankaRobot.collisionThresholds);
    jointImpedance = std::move(otherSimulinkFrankaRobot.jointImpedance);
    cartesianImpedance = std::move(otherSimulinkFrankaRobot.cartesianImpedance);
    loadInertia = std::move(otherSimulinkFrankaRobot.loadInertia);
    NE_T_EE = std::move(otherSimulinkFrankaRobot.NE_T_EE);
    EE_T_K = std::move(otherSimulinkFrankaRobot.EE_T_K);
    initialJointConfiguration = std::move(otherSimulinkFrankaRobot.initialJointConfiguration);
    
    controlThreadHasBeenSpawned = false;    
    controlThreadStepDone = false;
    terminateControlThread = false; 
    controlThreadException = false;
    inputsHaveBeenCopied = false;
    
    bool computationalPriorityIsFigured = false;
    
    currentlyInFirstControlStep = true;
    
    setControlModeMembersBegin();
    
    robot = std::make_unique<franka::Robot>(robotIPString);
    
    robotModel = std::make_unique<franka::Model>(robot->loadModel());
    
    return *this;
    
}

SimulinkFrankaRobot::~SimulinkFrankaRobot(){
    
    terminateControlThread = true;
    controlThreadStepDone = true;

    // notify control thread that the inputs have been read
    notifyControlThreadToContinue();
    
};
                                                     
void SimulinkFrankaRobot::parseMaskSettings(const char * robotIPmask,
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
                                           const double cutoffFrequencyMask){
    
    // robot_ip_
    robotIPString.assign(robotIPmask);
    
    // gripperAttached
    gripperAttached = gripperAttachedMask;
  
    // control mode
    controlMode = controlModeMask;

    // controller_choice
    controllerMode = (franka::ControllerMode)controllerChoiceMask;

    // collision_thresholds
    std::copy(collisionThresholdsMask,
              collisionThresholdsMask + 52, collisionThresholds.data());
   
    // joint_impedance
    std::copy(jointImpedanceMask,
              jointImpedanceMask + 7, jointImpedance.data());

    // cartesian_impedance
    std::copy(cartesianImpedanceMask,
              cartesianImpedanceMask + 6, cartesianImpedance.data());

    // load_inertia
    std::copy(loadInertiaMask,
              loadInertiaMask + 13, loadInertia.data());
    
    // NE_T_EE
    std::copy(NE_T_EEMask,
              NE_T_EEMask + 16, NE_T_EE.data());
    
    // EE_T_K
    std::copy(EE_T_KMask,
              EE_T_KMask + 16, EE_T_K.data());

    // init_joint_configuration_req
    initialJointConfigurationRequested = initialJointConfigurationRequestedMask;

    // init_joint_configuration
    std::copy(initialJointConfigurationMask,
              initialJointConfigurationMask + 7, initialJointConfiguration.data());

    // elbow control requested in case of cartesian pose or velocity is commanded.
    elbowControl = elbowControlMask;

    // rate limiter 
    rateLimiter = (bool)rateLimiterMask;

    //cutoff frequency 
    cutoffFrequency = (double)cutoffFrequencyMask;

    // control mode
    if ((bool)elbowControl) {
        switch (controlMode) {
          case torqueControlCartesianPose:
            controlMode = torqueControlCartesianPoseWithElbow;
            break;

          case torqueControlCartesianVelocities:
            controlMode = torqueControlCartesianVelocitiesWithElbow;
            break;

          case cartesianPoseControllerMode:
            controlMode = cartesianPoseControllerModeWithElbow;
            break;

          case cartesianVelocitiesControllerMode:
            controlMode = cartesianVelocitiesControllerModeWithElbow;
            break;
        }
    }
    
}

void SimulinkFrankaRobot::applyCollisionBehaviorSettings() {
    
    std::array<double, 7> lowerTorqueThresholdsAcceleration;
    std::copy_n(collisionThresholds.begin(),7, lowerTorqueThresholdsAcceleration.begin());

    std::array<double, 7> upperTorqueThresholdsAcceleration;
    std::copy_n(collisionThresholds.begin() + 7, 7, upperTorqueThresholdsAcceleration.begin());

    std::array<double, 7> lowerTorqueThresholdsNominal;
    std::copy_n(collisionThresholds.begin() + 14, 7, lowerTorqueThresholdsNominal.begin());

    std::array<double, 7> upperTorqueThresholdsNominal;
    std::copy_n(collisionThresholds.begin() + 21, 7, upperTorqueThresholdsNominal.begin());

    std::array<double, 6> lowerForceThresholdsAcceleration;
    std::copy_n(collisionThresholds.begin() + 28, 6, lowerForceThresholdsAcceleration.begin());

    std::array<double, 6> upperForceThresholdsAcceleration;
    std::copy_n(collisionThresholds.begin() + 34, 6, upperForceThresholdsAcceleration.begin());

    std::array<double, 6> lowerForceThresholdsNominal;
    std::copy_n(collisionThresholds.begin() + 40, 6, lowerForceThresholdsNominal.begin());

    std::array<double, 6> upperForceThresholdsNominal;
    std::copy_n(collisionThresholds.begin() + 46, 6, upperForceThresholdsNominal.begin());
    
    robot->setCollisionBehavior(
        lowerTorqueThresholdsAcceleration, 
        upperTorqueThresholdsAcceleration,
        lowerTorqueThresholdsNominal, 
        upperTorqueThresholdsNominal, 
        lowerForceThresholdsAcceleration,
        upperForceThresholdsAcceleration, 
        lowerForceThresholdsNominal, 
        upperForceThresholdsNominal);
}

void SimulinkFrankaRobot::applyRobotSettings() {
    // Is Initial Configuration Requested
    if (initialJointConfigurationRequested) {
        setDefaultBehavior(*robot);   
        MotionGenerator motionGenerator(0.5, initialJointConfiguration);
        robot->control(motionGenerator);
    }

  // Load Robot Parameters

  applyCollisionBehaviorSettings();
  robot->setJointImpedance(jointImpedance);
  robot->setCartesianImpedance(cartesianImpedance);
  robot->setEE(NE_T_EE);
  robot->setK(EE_T_K);
  std::array<double, 3> loadCenterOfMass;
  std::copy(loadInertia.begin() + 1, loadInertia.begin() + 4, loadCenterOfMass.data());
  std::array<double, 9> inertiaMatrix;
  std::copy(loadInertia.begin() + 4, loadInertia.begin() + 13, inertiaMatrix.data());
  robot->setLoad(loadInertia[0], loadCenterOfMass, inertiaMatrix);
}

// Map correlating strings reffering to robot state members with their corresponding vector sizes.
const double* SimulinkFrankaRobot::getRobotStateSignalPointerForStringLiteral(const std::string& signalNameString) {
   
        if (signalNameString.compare("O_T_EE") == 0)
            return robotState.O_T_EE.data();
        else if (signalNameString.compare("O_T_EE_d") == 0)
            return robotState.O_T_EE_d.data();
        else if (signalNameString.compare("F_T_EE") == 0)
            return robotState.F_T_EE.data();
        else if (signalNameString.compare("NE_T_EE") == 0)
            return robotState.NE_T_EE.data();
        else if (signalNameString.compare("EE_T_K") == 0)
            return robotState.EE_T_K.data();
        else if (signalNameString.compare("m_ee") == 0)
            return &robotState.m_ee;
        else if (signalNameString.compare("I_ee") == 0)
            return robotState.I_ee.data();
        else if (signalNameString.compare("F_x_Cee") == 0)
            return robotState.F_x_Cee.data();
        else if (signalNameString.compare("m_load") == 0)
            return &robotState.m_load;
        else if (signalNameString.compare("I_load") == 0)
            return robotState.I_load.data();
        else if (signalNameString.compare("F_x_Cload") == 0)
            return robotState.F_x_Cload.data();
        else if (signalNameString.compare("m_total") == 0)
            return &robotState.m_total;
        else if (signalNameString.compare("I_total") == 0)
            return robotState.I_total.data();
        else if (signalNameString.compare("F_x_Ctotal") == 0)
            return robotState.F_x_Ctotal.data();
        else if (signalNameString.compare("elbow") == 0)
            return robotState.elbow.data();
        else if (signalNameString.compare("elbow_d") == 0)
            return robotState.elbow_d.data();
        else if (signalNameString.compare("elbow_c") == 0)
            return robotState.elbow_c.data();
        else if (signalNameString.compare("delbow_c") == 0)
            return robotState.delbow_c.data();         
        else if (signalNameString.compare("ddelbow_c") == 0)
            return robotState.ddelbow_c.data();
        else if (signalNameString.compare("tau_J") == 0)
            return robotState.tau_J.data();
        else if (signalNameString.compare("tau_J_d") == 0)
            return robotState.tau_J_d.data();
        else if (signalNameString.compare("dtau_J") == 0)
            return robotState.dtau_J.data();
        else if (signalNameString.compare("q") == 0)
            return robotState.q.data();
        else if (signalNameString.compare("q_d") == 0)
            return robotState.q_d.data();
        else if (signalNameString.compare("dq") == 0)
            return robotState.dq.data();
        else if (signalNameString.compare("dq_d") == 0)
            return robotState.dq_d.data();
        else if (signalNameString.compare("ddq_d") == 0)
            return robotState.ddq_d.data();
        else if (signalNameString.compare("joint_contact") == 0)
            return robotState.joint_contact.data();
        else if (signalNameString.compare("cartesian_contact") == 0)
            return robotState.cartesian_contact.data();
        else if (signalNameString.compare("joint_collision") == 0)
            return robotState.joint_collision.data();
        else if (signalNameString.compare("cartesian_collision") == 0)
            return robotState.cartesian_collision.data();
        else if (signalNameString.compare("tau_ext_hat_filtered") == 0)
            return robotState.tau_ext_hat_filtered.data();
        else if (signalNameString.compare("O_F_ext_hat_K") == 0)
            return robotState.O_F_ext_hat_K.data();
        else if (signalNameString.compare("K_F_ext_hat_K") == 0)
            return robotState.K_F_ext_hat_K.data();
        else if (signalNameString.compare("O_dP_EE_d") == 0)
            return robotState.O_dP_EE_d.data();
        else if (signalNameString.compare("O_T_EE_c") == 0)
            return robotState.O_T_EE_c.data();
        else if (signalNameString.compare("O_dP_EE_c") == 0)
            return robotState.O_dP_EE_c.data();
        else if (signalNameString.compare("O_ddP_EE_c") == 0)
            return robotState.O_ddP_EE_c.data();
        else if (signalNameString.compare("theta") == 0)
            return robotState.theta.data();
        else if (signalNameString.compare("dtheta") == 0)
            return robotState.dtheta.data();     
        else if (signalNameString.compare("control_command_success_rate") == 0)
            return &robotState.control_command_success_rate;            
}

void SimulinkFrankaRobot::setControlModeMembersBegin() {
    
        controlModeMembersBegin[0].memberBegin[0] = controllerInputs.tau_J_d.data();
        
        controlModeMembersBegin[1].memberBegin[0] = controllerInputs.q_d.data();
        controlModeMembersBegin[1].memberBegin[1] = controllerInputs.tau_J_d.data();
        
        controlModeMembersBegin[2].memberBegin[0] = controllerInputs.q_d.data();
        controlModeMembersBegin[2].memberBegin[1] = controllerInputs.tau_J_d.data();
            
        controlModeMembersBegin[3].memberBegin[0] = controllerInputs.O_T_EE_d.data();
        controlModeMembersBegin[3].memberBegin[1] = controllerInputs.tau_J_d.data();
        
        controlModeMembersBegin[4].memberBegin[0] = controllerInputs.O_dP_EE_d.data();
        controlModeMembersBegin[4].memberBegin[1] = controllerInputs.tau_J_d.data();

        controlModeMembersBegin[5].memberBegin[0] = controllerInputs.q_d.data();
        
        controlModeMembersBegin[6].memberBegin[0] = controllerInputs.dq_d.data();
        
        controlModeMembersBegin[7].memberBegin[0] = controllerInputs.O_T_EE_d.data();
        
        controlModeMembersBegin[8].memberBegin[0] = controllerInputs.O_dP_EE_d.data();
        
        controlModeMembersBegin[9].memberBegin[0] = controllerInputs.O_T_EE_d.data();
        controlModeMembersBegin[9].memberBegin[1] = controllerInputs.elbow_d.data();
        controlModeMembersBegin[9].memberBegin[2] = controllerInputs.tau_J_d.data();
        
        controlModeMembersBegin[10].memberBegin[0] = controllerInputs.O_dP_EE_d.data();
        controlModeMembersBegin[10].memberBegin[1] = controllerInputs.elbow_d.data();
        controlModeMembersBegin[10].memberBegin[2] = controllerInputs.tau_J_d.data();
        
        controlModeMembersBegin[11].memberBegin[0] = controllerInputs.O_T_EE_d.data();
        controlModeMembersBegin[11].memberBegin[1] = controllerInputs.elbow_d.data();
        
        controlModeMembersBegin[12].memberBegin[0] = controllerInputs.O_dP_EE_d.data();
        controlModeMembersBegin[12].memberBegin[1] = controllerInputs.elbow_d.data();
        
}

int SimulinkFrankaRobot::establishIfCurrentBlockFirstToBeComputed(){
    if (!computationalPriorityIsFigured) {
        computationalPriorityIsFigured = true;
        return 1;
    } else {
        return 0;
    }
}

void SimulinkFrankaRobot::checkIfAndHandleException(){  
    if (controlThreadException) {
      throw std::runtime_error("Execution terminated due to exception thrown during robot control!\n");
      return;
    }
}

void SimulinkFrankaRobot::notifyControlThreadToContinue(){
    if (!currentlyInFirstControlStep) {
        std::unique_lock<std::mutex> lk2(mutexForEnsuringCurrentStepStepControlSignal);
        inputsHaveBeenCopied = true;
        lk2.unlock();
        conditionVariableForEnsuringCurrentStepStepControlSignal.notify_one();
    }
}

void SimulinkFrankaRobot::waitForControlThreadStep(){ 
    std::unique_lock<std::mutex> lk(mutexForMainControlStepCycle);
    conditionVariableForMainControlStepCycle.wait(lk, [&] { return controlThreadStepDone; });
    controlThreadStepDone = false;
}

void SimulinkFrankaRobot::notifyMainThreadToContinue()
{
    std::unique_lock<std::mutex> lk(mutexForMainControlStepCycle);
    controlThreadStepDone = true;
    lk.unlock();
    conditionVariableForMainControlStepCycle.notify_one(); 
}

void SimulinkFrankaRobot::waitNotificationFromMainThreadToContinue()
{
    std::unique_lock<std::mutex> lk2(mutexForEnsuringCurrentStepStepControlSignal);
    conditionVariableForEnsuringCurrentStepStepControlSignal.wait(lk2, [&]{return inputsHaveBeenCopied;});
    inputsHaveBeenCopied = false;
}

extern void (*controlModeCallbackFunction[13])(SimulinkFrankaRobot& simulinkFrankaRobot);

void SimulinkFrankaRobot::spawnControlThread(){
    
    // Spawn the corresponding control thread
    controlThread = std::thread(controlModeCallbackFunction[controlMode], std::ref(*this));

//     sched_param sch;
//     int policy;
//     pthread_getschedparam(controlThread.native_handle(), &policy, &sch);
//     sch.sched_priority = 1;
//     if (pthread_setschedparam(controlThread.native_handle(), SCHED_FIFO, &sch)) {
//       throw std::runtime_error("Failure to apply control thread configuration!\n");
//       return;
//     }
    controlThread.detach();
    controlThreadHasBeenSpawned = true;
}

void SimulinkFrankaRobot::copyInputSignal(const double *inputSignal, const int inputSignalIndex){
    
     std::copy(inputSignal,
               inputSignal + controlModeSizes[controlMode].width[inputSignalIndex - gripperAttached],
               (double*)controlModeMembersBegin[controlMode].memberBegin[inputSignalIndex - gripperAttached]);
     
}

void SimulinkFrankaRobot::copyOutputSignal(std::vector<std::string>& outputSignalsStack, int outputSignalIndex, double *outputSignal ){
    std::copy(
          getRobotStateSignalPointerForStringLiteral(outputSignalsStack.at(outputSignalIndex)),
          getRobotStateSignalPointerForStringLiteral(outputSignalsStack.at(outputSignalIndex)) + getWidthForSignalString[outputSignalsStack.at(outputSignalIndex)],
          outputSignal);
}

bool SimulinkFrankaRobot::getControlThreadHasBeenSpawned() const{
    return controlThreadHasBeenSpawned;
}

void SimulinkFrankaRobot::setSampleTime(double currentSampleTime){
    sampleTime = currentSampleTime;
}

double SimulinkFrankaRobot::getSampleTime() const{
    return sampleTime;
}

int SimulinkFrankaRobot::getElbowControl() const{
    return elbowControl;
}

void SimulinkFrankaRobot::mountRobotState(const franka::RobotState& currentRobotState){
    robotState = currentRobotState;
}

const franka::RobotState SimulinkFrankaRobot::getRobotState(){
    return robotState;
}

void SimulinkFrankaRobot::copyRobotPose(int frame, double *outputSignal){
    franka::Frame frame_ = static_cast<franka::Frame>(frame-1);
    std::array<double, 16> pose = robotModel->pose(frame_, robotState);
    std::copy(pose.begin(), pose.end(), outputSignal);
}

void SimulinkFrankaRobot::copyRobotPose(int frame,
                                        const double *q,
                                        const double *F_T_EE,
                                        const double *EE_T_K,
                                        double *outputSignal){
                                            
    franka::Frame frame_ = static_cast<franka::Frame>(frame-1);

    std::array< double, 7 > q_;
    std::copy(q, q + 7, q_.begin());

    std::array< double, 16 > F_T_EE_;
    std::copy(F_T_EE, F_T_EE + 16, F_T_EE_.begin());

    std::array< double, 16 > EE_T_K_;
    std::copy(EE_T_K, EE_T_K + 16, EE_T_K_.begin());

    std::array<double, 16> pose = robotModel->pose(frame_, q_, F_T_EE_, EE_T_K_);
    std::copy(pose.begin(), pose.end(), outputSignal);
}

void SimulinkFrankaRobot::copyJacobian( int type, 
                                        int frame, 
                                        double *outputSignal){
    
    franka::Frame frame_ = static_cast<franka::Frame>(frame-1);
    
    std::array<double, 42> jac;
    
    if (type == 1){
        jac = robotModel->zeroJacobian(frame_, robotState);
    }else if (type == 2){
        jac = robotModel->bodyJacobian(frame_, robotState);
    }
    std::copy(jac.begin(), jac.end(), outputSignal);
}


void SimulinkFrankaRobot::copyJacobian( int type, 
                                        int frame,                        
                                        const double *q,
                                        const double *F_T_EE,
                                        const double *EE_T_K,
                                        double *outputSignal){
    
    franka::Frame frame_ = static_cast<franka::Frame>(frame-1);
    
    std::array< double, 7 > q_;
    std::copy(q, q + 7, q_.begin());

    std::array< double, 16 > F_T_EE_;
    std::copy(F_T_EE, F_T_EE + 16, F_T_EE_.begin());

    std::array< double, 16 > EE_T_K_;
    std::copy(EE_T_K, EE_T_K + 16, EE_T_K_.begin());
    
    std::array<double, 42> jac;
    
    if (type == 1){
        jac = robotModel->zeroJacobian(frame_, q_, F_T_EE_, EE_T_K_);
    }else if (type == 2){
        jac = robotModel->bodyJacobian(frame_, q_, F_T_EE_, EE_T_K_);
    }
    std::copy(jac.begin(), jac.end(), outputSignal);
}

void SimulinkFrankaRobot::copyMass(double *outputSignal){
    std::array<double, 49> mass = robotModel->mass(robotState);
    std::copy(mass.begin(), mass.end(), outputSignal);
}

void SimulinkFrankaRobot::copyMass( const double *q,
                                    const double *I_total,
                                    const double *m_total,
                                    const double *F_x_Ctotal,
                                    double *outputSignal){
    
    std::array< double, 7 > q_;
    std::copy(q, q + 7, q_.begin());
    std::array< double, 9 > I_total_;
    std::copy(I_total, I_total + 9, I_total_.begin());
    std::array< double, 3 > F_x_Ctotal_;
    std::copy(F_x_Ctotal, F_x_Ctotal + 3, F_x_Ctotal_.begin());

    std::array<double, 49> mass = robotModel->mass(q_, I_total_, *m_total, F_x_Ctotal_);
    std::copy(mass.begin(), mass.end(), outputSignal);
}

void SimulinkFrankaRobot::copyCoriolis(double *outputSignal){
    std::array<double, 7> cor = robotModel->coriolis(robotState);
    std::copy(cor.begin(), cor.end(), outputSignal);
}

void SimulinkFrankaRobot::copyCoriolis( const double *q,
                                        const double *dq,
                                        const double *I_total,
                                        const double *m_total,
                                        const double *F_x_Ctotal,
                                        double *outputSignal){
    
    std::array< double, 7 > q_;
    std::copy(q, q + 7, q_.begin());
    std::array< double, 7 > dq_;
    std::copy(dq, dq + 7, dq_.begin());
    std::array< double, 9 > I_total_;
    std::copy(I_total, I_total + 9, I_total_.begin());
    std::array< double, 3 > F_x_Ctotal_;
    std::copy(F_x_Ctotal, F_x_Ctotal + 3, F_x_Ctotal_.begin());

    std::array<double, 7> cor = robotModel->coriolis(q_, dq_, I_total_, *m_total, F_x_Ctotal_);
    std::copy(cor.begin(), cor.end(), outputSignal);
}

void SimulinkFrankaRobot::copyGravity(double *outputSignal){
    std::array<double, 7> grav = robotModel->gravity(robotState);
    std::copy(grav.begin(), grav.end(), outputSignal);
}

void SimulinkFrankaRobot::copyGravity(const double *gravity_earth, 
                                      double *outputSignal){

    std::array< double, 3 > gravity_earth_;
    std::copy(gravity_earth, gravity_earth + 3, gravity_earth_.begin());

    std::array<double, 7> grav = robotModel->gravity(robotState, gravity_earth_);
    std::copy(grav.begin(), grav.end(), outputSignal);
}

void SimulinkFrankaRobot::copyGravity(const double *q,
                                      const double *m_total,
                                      const double *F_x_Ctotal, 
                                      double *outputSignal){

    std::array< double, 7 > q_;
    std::copy(q, q + 7, q_.begin());
    std::array< double, 3 > F_x_Ctotal_;
    std::copy(F_x_Ctotal, F_x_Ctotal + 3, F_x_Ctotal_.begin());

    std::array<double, 7> grav = robotModel->gravity(q_, *m_total, F_x_Ctotal_);
    std::copy(grav.begin(), grav.end(), outputSignal);
}

void SimulinkFrankaRobot::copyGravity(const double *q,
                                      const double *m_total,
                                      const double *F_x_Ctotal,
                                      const double *gravity_earth, 
                                      double *outputSignal){

    std::array< double, 7 > q_;
    std::copy(q, q + 7, q_.begin());
    std::array< double, 3 > F_x_Ctotal_;
    std::copy(F_x_Ctotal, F_x_Ctotal + 3, F_x_Ctotal_.begin());
    std::array< double, 3 > gravity_earth_;
    std::copy(gravity_earth, gravity_earth + 3, gravity_earth_.begin());

    std::array<double, 7> grav = robotModel->gravity(q_, *m_total, F_x_Ctotal_, gravity_earth_);
    std::copy(grav.begin(), grav.end(), outputSignal);
}

void SimulinkFrankaRobot::setCurrentlyInFirstControlStep(bool controlStepFlag){
    currentlyInFirstControlStep = controlStepFlag;
}

bool SimulinkFrankaRobot::getCurrentlyInFirstControlStep() const{
    return currentlyInFirstControlStep;
}

bool SimulinkFrankaRobot::getTerminateControlThread() const{
    return terminateControlThread;
}

franka::ControllerMode SimulinkFrankaRobot::getControllerMode() const{
    return controllerMode;
}

bool SimulinkFrankaRobot::getRateLimiter() const{
    return rateLimiter;
}

double SimulinkFrankaRobot::getCutoffFrequency() const{
    return cutoffFrequency;
}

void SimulinkFrankaRobot::setControlThreadExceptionFlag(){
    controlThreadException = true; 
};

std::array<double, 7> const &SimulinkFrankaRobot::getControllerInputTorques() const{
    return controllerInputs.tau_J_d;
};
std::array<double, 7> const &SimulinkFrankaRobot::getControllerInputJointPositions() const{
    return controllerInputs.q_d;
};
std::array<double, 7> const &SimulinkFrankaRobot::getControllerInputJointVelocities() const{
    return controllerInputs.dq_d;
};
std::array<double, 16> const &SimulinkFrankaRobot::getControllerInputCartesianPose() const{
    return controllerInputs.O_T_EE_d;
};
std::array<double, 6> const &SimulinkFrankaRobot::getControllerInputCartesianVelocities() const{
    return controllerInputs.O_dP_EE_d;
};
std::array<double, 2> const &SimulinkFrankaRobot::getControllerInputElbowPosition() const{
    return controllerInputs.elbow_d;
};
