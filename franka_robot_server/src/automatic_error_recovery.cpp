#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>

kj::Promise<void> FrankaRobotRPCServiceImpl::automaticErrorRecovery(
    capnp::CallContext<AutomaticErrorRecoveryParams, AutomaticErrorRecoveryResults> context) {
    robot_->automaticErrorRecovery();
    
    return kj::READY_NOW;
}