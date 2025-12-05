#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <array>

kj::Promise<void> FrankaRobotRPCServiceImpl::setGuidingMode(
    capnp::CallContext<SetGuidingModeParams, SetGuidingModeResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto guiding_mode = params.getGuidingMode();
    bool elbow = params.getElbow();

    if (guiding_mode.size() != 6) {
        KJ_FAIL_REQUIRE("Guiding mode array must have exactly 6 elements");
    }

    try {
        // Convert to std::array
        std::array<bool, 6> guiding_mode_arr{};
        for (size_t i = 0; i < 6; i++) {
            guiding_mode_arr[i] = guiding_mode[i];
        }

        // Set guiding mode
        robot_->setGuidingMode(guiding_mode_arr, elbow);

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set guiding mode", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

