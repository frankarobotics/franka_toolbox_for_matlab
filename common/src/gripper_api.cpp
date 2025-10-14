//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#include "gripper_api.h"
#include <franka/gripper.h>

SimulinkFrankaGripper::SimulinkFrankaGripper(const char * robotIPMask){
    
  robotIP.assign(robotIPMask); 
  
  terminateGripperThread = false;
  gripperCurrentControlMode = idle_;
  
  gripper = std::make_unique<franka::Gripper>(robotIP);
  gripperControlThread = std::thread(&gripperControlThread_func, std::ref(*this));
  
  // toggle first time to notify the sim model that the gripper is ready to receive commands
  updateCommandCounter();
  {
    std::unique_lock<std::mutex> lk(m);
    gripperCommandApplied = true;
    lk.unlock();
    cv.notify_one();
  }
}

bool SimulinkFrankaGripper::gripperControlRequested() {
  bool current = triggerGripperCommand;
  if (current && !triggerGripperCommandPrev) {  // positive edge
    triggerGripperCommandPrev = current;
    return true;
  }
  triggerGripperCommandPrev = current;
  return false;
}

SimulinkFrankaGripper::~SimulinkFrankaGripper(){
    
  terminateGripperThread = true;

  gripperCurrentControlMode = idle_;
  // notify gripper control thread that the inputs have been read
  {
    std::unique_lock<std::mutex> lk(m);
    gripperCommandApplied = true;
    lk.unlock();
    cv.notify_one();
  }

  if (gripperControlThread.joinable()) {
    gripperControlThread.join();
  }       
}

void SimulinkFrankaGripper::parseGripperCommand(double *gripperCommandVector){
    
  gripperCurrentControlMode = static_cast<int>(gripperCommandVector[0]);
  triggerGripperCommand = static_cast<int>(gripperCommandVector[1]);
  graspWidth = static_cast<double>(gripperCommandVector[2]);
  graspSpeed = static_cast<double>(gripperCommandVector[3]);
  graspForce = static_cast<double>(gripperCommandVector[4]);
  graspEpsilonInner = static_cast<double>(gripperCommandVector[5]);
  graspEpsilonOuter = static_cast<double>(gripperCommandVector[6]);
  moveWidth = static_cast<double>(gripperCommandVector[7]);
  moveSpeed = static_cast<double>(gripperCommandVector[8]);
  
}

void SimulinkFrankaGripper::applyGripperCommand(){
    
  if (gripperControlRequested()) {
    {
      std::unique_lock<std::mutex> lk(m);
      gripperCommandApplied = true;
      lk.unlock();
      cv.notify_one();
    }
  }
}

void SimulinkFrankaGripper::waitForGripperCommand(){
  std::unique_lock<std::mutex> lk(m);
  cv.wait(lk, [&] { return gripperCommandApplied; });
  gripperCommandApplied = false;
}

double SimulinkFrankaGripper::getGripperWidth(){
  return gripperState.width;
}
double SimulinkFrankaGripper::getGripperMaxWidth(){
  return gripperState.max_width;
}
double SimulinkFrankaGripper::getGripperIsGrasped(){
  return gripperState.is_grasped;
}
double SimulinkFrankaGripper::getGripperGraspFailed(){
  return gripperGraspFailed;
}
double SimulinkFrankaGripper::getGripperTemperature(){
  return gripperState.temperature;
}
int SimulinkFrankaGripper::getGripperStateNewlyRead(){
  return stateNewlyReadCounter;
}
bool SimulinkFrankaGripper::getTerminateGripperThread(){
  return terminateGripperThread;
}
int SimulinkFrankaGripper::getGripperCurrentControlMode(){
  return gripperCurrentControlMode;
}

void SimulinkFrankaGripper::updateGripperState(){
  gripperState = gripper->readOnce();
}

void SimulinkFrankaGripper::performHoming(){
  gripper->homing();
}

void SimulinkFrankaGripper::performGrasp(){
  if (!gripper->grasp(graspWidth, graspSpeed, graspForce, graspEpsilonInner,
                      graspEpsilonOuter)) {
    gripperGraspFailed = 1;
  } else {
    gripperGraspFailed = 0;
  }
}

void SimulinkFrankaGripper::performMove(){
  gripper->move(moveWidth, moveSpeed);
}

void SimulinkFrankaGripper::performStop(){
  gripper->stop();
}

void SimulinkFrankaGripper::equalizeCommandCounter(){
    stateNewlyReadCounterPrev = stateNewlyReadCounter;
}

void SimulinkFrankaGripper::updateCommandCounter(){
  stateNewlyReadCounter++;
}

bool SimulinkFrankaGripper::gripperServerBusy(){
  return (stateNewlyReadCounterPrev == stateNewlyReadCounter);
}

void gripperControlThread_func(SimulinkFrankaGripper& simulinkFrankaGripper) {
  while (!simulinkFrankaGripper.getTerminateGripperThread()) {
    simulinkFrankaGripper.waitForGripperCommand();
    simulinkFrankaGripper.equalizeCommandCounter();

    switch (simulinkFrankaGripper.getGripperCurrentControlMode()) {
      case idle_:
        break;

      case read_state_:
        simulinkFrankaGripper.updateGripperState();
        break;

      case homing_:
        simulinkFrankaGripper.performHoming();
        break;

      case grasp_:
        simulinkFrankaGripper.performGrasp();
        break;

      case move_:
        simulinkFrankaGripper.performMove();
        break;

      case stop_:
        simulinkFrankaGripper.performStop();
        break;
    }

    simulinkFrankaGripper.updateCommandCounter();
  }
}