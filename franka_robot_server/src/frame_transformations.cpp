#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <array>

kj::Promise<void> FrankaRobotRPCServiceImpl::setK(
    capnp::CallContext<SetKParams, SetKResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto ee_t_k = params.getEeTK();

    if (ee_t_k.size() != 16) {
        KJ_FAIL_REQUIRE("EE_T_K transformation matrix must have exactly 16 elements");
    }

    try {
        // Convert to std::array
        std::array<double, 16> EE_T_K{};
        for (size_t i = 0; i < 16; i++) {
            EE_T_K[i] = ee_t_k[i];
        }

        // Set stiffness frame transformation
        robot_->setK(EE_T_K);

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set EE_T_K transformation", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::setEE(
    capnp::CallContext<SetEEParams, SetEEResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto ne_t_ee = params.getNeTEe();

    if (ne_t_ee.size() != 16) {
        KJ_FAIL_REQUIRE("NE_T_EE transformation matrix must have exactly 16 elements");
    }

    try {
        // Convert to std::array
        std::array<double, 16> NE_T_EE{};
        for (size_t i = 0; i < 16; i++) {
            NE_T_EE[i] = ne_t_ee[i];
        }

        // Set end effector frame transformation
        robot_->setEE(NE_T_EE);

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set NE_T_EE transformation", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

