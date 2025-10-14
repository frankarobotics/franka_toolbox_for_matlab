//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package
#include "vacuum_gripper_api.h"
#include <franka/gripper.h>
#include <franka/exception.h>
#include <chrono>

SimulinkFrankaVacuumGripper::SimulinkFrankaVacuumGripper(const char * robotIPMask){
    
  robotIP.assign(robotIPMask); 
  
  terminateGripperThread = false;
  gripperCurrentControlMode = idle_;
  
  gripper = std::make_unique<franka::VacuumGripper>(robotIP);
  gripperControlThread = std::thread(&vacuumGripperControlThread_func, std::ref(*this));
  
  // toggle first time to notify the sim model that the gripper is ready to receive commands
  updateCommandCounter();
  {
    std::unique_lock<std::mutex> lk(m);
    gripperCommandApplied = true;
    lk.unlock();
    cv.notify_one();
  }
}

bool SimulinkFrankaVacuumGripper::gripperControlRequested() {
  bool current = triggerGripperCommand;
  if (current && !triggerGripperCommandPrev) {  // positive edge
    triggerGripperCommandPrev = current;
    return true;
  }
  triggerGripperCommandPrev = current;
  return false;
}

SimulinkFrankaVacuumGripper::~SimulinkFrankaVacuumGripper(){
    
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

void SimulinkFrankaVacuumGripper::parseGripperCommand(double *gripperCommandVector){
    
  gripperCurrentControlMode = static_cast<int>(gripperCommandVector[0]);
  triggerGripperCommand = static_cast<int>(gripperCommandVector[1]);
  vacuum_control_point = static_cast<uint8_t>(gripperCommandVector[2]);
  vacuum_timeout = std::chrono::milliseconds(static_cast<int>(gripperCommandVector[3]));
  vacuum_profile = static_cast<franka::VacuumGripper::ProductionSetupProfile>(static_cast<int>(gripperCommandVector[4]));
  vacuum_timeout_drop_off = std::chrono::milliseconds(static_cast<int>(gripperCommandVector[5]));
}

void SimulinkFrankaVacuumGripper::applyGripperCommand(){
    
  if (gripperControlRequested()) {
    {
      std::unique_lock<std::mutex> lk(m);
      gripperCommandApplied = true;
      lk.unlock();
      cv.notify_one();
    }
  }
}

void SimulinkFrankaVacuumGripper::waitForGripperCommand(){
  std::unique_lock<std::mutex> lk(m);
  cv.wait(lk, [&] { return gripperCommandApplied; });
  gripperCommandApplied = false;
}

int SimulinkFrankaVacuumGripper::getGripperStateNewlyRead(){
  return stateNewlyReadCounter;
}

bool SimulinkFrankaVacuumGripper::getTerminateGripperThread(){
  return terminateGripperThread;
}

int SimulinkFrankaVacuumGripper::getGripperCurrentControlMode(){
  return gripperCurrentControlMode;
}

void SimulinkFrankaVacuumGripper::updateGripperState(){
  vacuumGripperState = gripper->readOnce();
}

void SimulinkFrankaVacuumGripper::performVacuum() {
  if (gripper->vacuum(vacuum_control_point, vacuum_timeout, vacuum_profile)) {
    vacuumEstablished = 1;
  } else {
    vacuumEstablished = 0;
  }
}

void SimulinkFrankaVacuumGripper::performDropOff() {
  gripper->dropOff(vacuum_timeout_drop_off);
}

void SimulinkFrankaVacuumGripper::performStop(){
  gripper->stop();
}

void SimulinkFrankaVacuumGripper::equalizeCommandCounter(){
    stateNewlyReadCounterPrev = stateNewlyReadCounter;
}

void SimulinkFrankaVacuumGripper::updateCommandCounter(){
  stateNewlyReadCounter++;
}

bool SimulinkFrankaVacuumGripper::gripperServerBusy(){
  return (stateNewlyReadCounterPrev == stateNewlyReadCounter);
}

bool SimulinkFrankaVacuumGripper::getInControlRange() {
  return vacuumGripperState.in_control_range;
}

bool SimulinkFrankaVacuumGripper::getPartDetached() {
  return vacuumGripperState.part_detached;
}

bool SimulinkFrankaVacuumGripper::getPartPresent() {
  return vacuumGripperState.part_present;
}

uint8_t SimulinkFrankaVacuumGripper::getDeviceStatus() {
  return static_cast<uint8_t>(vacuumGripperState.device_status);
}

uint16_t SimulinkFrankaVacuumGripper::getCurrentActualPower() {
  return vacuumGripperState.actual_power;
}

uint16_t SimulinkFrankaVacuumGripper::getCurrentSystemVacuum() {
  return vacuumGripperState.vacuum;
}

franka::Duration SimulinkFrankaVacuumGripper::getTime() {
  return vacuumGripperState.time;
}

void vacuumGripperControlThread_func(SimulinkFrankaVacuumGripper& simulinkFrankaVacuumGripper) {
  while (!simulinkFrankaVacuumGripper.getTerminateGripperThread()) {
    simulinkFrankaVacuumGripper.waitForGripperCommand();
    simulinkFrankaVacuumGripper.equalizeCommandCounter();

    try {
      switch (simulinkFrankaVacuumGripper.getGripperCurrentControlMode()) {
        case idle_:
          break;

        case read_state_:
          simulinkFrankaVacuumGripper.updateGripperState();
          break;

        case vacuum_:
          simulinkFrankaVacuumGripper.performVacuum();
          break;

        case drop_off_:
          simulinkFrankaVacuumGripper.performDropOff();
          break;

        case stop_:
          simulinkFrankaVacuumGripper.performStop();
          break;
      }
    } catch (const franka::Exception& e) {
      std::cerr << "Vacuum gripper error: " << e.what() << std::endl;
    }

    simulinkFrankaVacuumGripper.updateCommandCounter();
  }
} 