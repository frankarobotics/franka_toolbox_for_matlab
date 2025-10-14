#pragma once

#include <capnp/ez-rpc.h>
#include "rpc.capnp.h"
#include <string>
#include <memory>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/vacuum_gripper.h>

class FrankaRobotRPCServiceImpl final : public RPCService::Server {
public:
    FrankaRobotRPCServiceImpl();

    kj::Promise<void> automaticErrorRecovery(
        capnp::CallContext<AutomaticErrorRecoveryParams, AutomaticErrorRecoveryResults> context) override;

    kj::Promise<void> initializeRobot(
        capnp::CallContext<InitializeRobotParams, InitializeRobotResults> context) override;

    kj::Promise<void> initializeGripper(
        capnp::CallContext<InitializeGripperParams, InitializeGripperResults> context) override;

    kj::Promise<void> initializeVacuumGripper(
        capnp::CallContext<InitializeVacuumGripperParams, InitializeVacuumGripperResults> context) override;

    kj::Promise<void> getRobotState(
        capnp::CallContext<GetRobotStateParams, GetRobotStateResults> context) override;

    kj::Promise<void> getJointPoses(
        capnp::CallContext<GetJointPosesParams, GetJointPosesResults> context) override;

    kj::Promise<void> jointPointToPointMotion(
        capnp::CallContext<JointPointToPointMotionParams, JointPointToPointMotionResults> context) override;

    kj::Promise<void> jointTrajectoryMotion(
        capnp::CallContext<JointTrajectoryMotionParams, JointTrajectoryMotionResults> context) override;

    kj::Promise<void> getGripperState(
        capnp::CallContext<GetGripperStateParams, GetGripperStateResults> context) override;

    kj::Promise<void> gripperGrasp(
        capnp::CallContext<GripperGraspParams, GripperGraspResults> context) override;

    kj::Promise<void> gripperHoming(
        capnp::CallContext<GripperHomingParams, GripperHomingResults> context) override;

    kj::Promise<void> gripperMove(
        capnp::CallContext<GripperMoveParams, GripperMoveResults> context) override;

    kj::Promise<void> gripperStop(
        capnp::CallContext<GripperStopParams, GripperStopResults> context) override;

    kj::Promise<void> setCollisionBehavior(
        capnp::CallContext<SetCollisionBehaviorParams, SetCollisionBehaviorResults> context) override;

    kj::Promise<void> setLoadInertia(
        capnp::CallContext<SetLoadInertiaParams, SetLoadInertiaResults> context) override;

    // Vacuum Gripper methods
    kj::Promise<void> getVacuumGripperState(
        capnp::CallContext<GetVacuumGripperStateParams, GetVacuumGripperStateResults> context) override;

    kj::Promise<void> vacuumGripperVacuum(
        capnp::CallContext<VacuumGripperVacuumParams, VacuumGripperVacuumResults> context) override;

    kj::Promise<void> vacuumGripperDropOff(
        capnp::CallContext<VacuumGripperDropOffParams, VacuumGripperDropOffResults> context) override;

    kj::Promise<void> vacuumGripperStop(
        capnp::CallContext<VacuumGripperStopParams, VacuumGripperStopResults> context) override;

private:
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    std::unique_ptr<franka::Gripper> gripper_;
    std::unique_ptr<franka::VacuumGripper> vacuum_gripper_;
    std::string robot_ip_;
}; 