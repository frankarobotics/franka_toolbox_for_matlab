#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <memory>

FrankaRobotRPCServiceImpl::FrankaRobotRPCServiceImpl() {
    // Do not initialize robot, model, gripper, or vacuum_gripper here.
    // Initialization will occur in initializeRobot().
}

kj::Promise<void> FrankaRobotRPCServiceImpl::initializeRobot(
    capnp::CallContext<InitializeRobotParams, InitializeRobotResults> context) {
    try {
        robot_ip_ = context.getParams().getIpAddress();
        robot_ = std::make_unique<franka::Robot>(robot_ip_);
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
        // gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        // vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip_);
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        // Optionally: context.getResults().setErrorMessage(e.what());
        KJ_LOG(ERROR, "Franka exception during robot/model initialization:", e.what());
        return kj::READY_NOW; // Or return a rejected promise if your framework supports it
    } catch (const std::exception& e) {
        KJ_LOG(ERROR, "Standard exception during robot/model initialization:", e.what());
        return kj::READY_NOW;
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::initializeGripper(
    capnp::CallContext<InitializeGripperParams, InitializeGripperResults> context) {
    try {
        gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Franka exception during gripper initialization:", e.what());
        return kj::READY_NOW;
    } catch (const std::exception& e) {
        KJ_LOG(ERROR, "Standard exception during gripper initialization:", e.what());
        return kj::READY_NOW;
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::initializeVacuumGripper(
    capnp::CallContext<InitializeVacuumGripperParams, InitializeVacuumGripperResults> context) {
    try {
        vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip_);
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Franka exception during vacuum gripper initialization:", e.what());
        return kj::READY_NOW;
    } catch (const std::exception& e) {
        KJ_LOG(ERROR, "Standard exception during vacuum gripper initialization:", e.what());
        return kj::READY_NOW;
    }
}