#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/gripper.h>
#include <memory>

kj::Promise<void> FrankaRobotRPCServiceImpl::getGripperState(
    capnp::CallContext<GetGripperStateParams, GetGripperStateResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        franka::GripperState state = gripper_->readOnce();

        auto results = context.getResults();
        auto gripper_state = results.initState();
        
        gripper_state.setWidth(state.width);
        gripper_state.setMaxWidth(state.max_width);
        gripper_state.setIsGrasped(state.is_grasped);
        gripper_state.setTemperature(state.temperature);
        gripper_state.setTimeStamp(state.time.toSec());

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to get gripper state", e.what());
        throw;
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperGrasp(
    capnp::CallContext<GripperGraspParams, GripperGraspResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double width = params.getWidth();
    double speed = params.getSpeed();
    double force = params.getForce();
    double epsilon_inner = params.getEpsilonInner();
    double epsilon_outer = params.getEpsilonOuter();

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        bool success = gripper_->grasp(width, speed, force, epsilon_inner, epsilon_outer);
        
        auto results = context.getResults();
        results.setSuccess(success);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper grasp failed", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperHoming(
    capnp::CallContext<GripperHomingParams, GripperHomingResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        bool success = gripper_->homing();
        
        auto results = context.getResults();
        results.setSuccess(success);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper homing failed", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperMove(
    capnp::CallContext<GripperMoveParams, GripperMoveResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double width = params.getWidth();
    double speed = params.getSpeed();

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        bool success = gripper_->move(width, speed);
        
        auto results = context.getResults();
        results.setSuccess(success);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper move failed", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperStop(
    capnp::CallContext<GripperStopParams, GripperStopResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        bool success = gripper_->stop();
        
        auto results = context.getResults();
        results.setSuccess(success);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper stop failed", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
} 