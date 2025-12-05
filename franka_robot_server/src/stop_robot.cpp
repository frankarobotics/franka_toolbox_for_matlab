#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>

kj::Promise<void> FrankaRobotRPCServiceImpl::stopRobot(
    capnp::CallContext<StopRobotParams, StopRobotResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        // Stop all currently running motions
        robot_->stop();

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to stop robot", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

