#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/control_types.h>
#include <array>
#include <cmath>
#include "franka_matlab_robot_utils.h"
#include <iostream>

kj::Promise<void> FrankaRobotRPCServiceImpl::jointPointToPointMotion(
    capnp::CallContext<JointPointToPointMotionParams, JointPointToPointMotionResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto target_config = params.getTargetConfiguration();
    double speed_factor = params.getSpeedFactor();

    if (target_config.size() != 7) {
        KJ_FAIL_REQUIRE("Target configuration must have exactly 7 joint angles");
    }

    if (speed_factor <= 0.0 || speed_factor > 1.0) {
        KJ_FAIL_REQUIRE("Speed factor must be in range (0, 1]");
    }

    try {
        std::array<double, 7> q_goal{};
        for (size_t i = 0; i < 7; i++) {
            q_goal[i] = target_config[i];
        }

        MotionGenerator motion_generator(speed_factor, q_goal);
        robot_->control(motion_generator);

        auto results = context.getResults();
        results.setResult(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Joint point-to-point motion failed", e.what());
        auto results = context.getResults();
        results.setResult(false);
    }

    return kj::READY_NOW;
} 