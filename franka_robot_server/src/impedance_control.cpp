#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <array>

kj::Promise<void> FrankaRobotRPCServiceImpl::setJointImpedance(
    capnp::CallContext<SetJointImpedanceParams, SetJointImpedanceResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto k_theta = params.getKTheta();

    if (k_theta.size() != 7) {
        KJ_FAIL_REQUIRE("Joint impedance array must have exactly 7 elements");
    }

    try {
        // Convert to std::array
        std::array<double, 7> K_theta{};
        for (size_t i = 0; i < 7; i++) {
            K_theta[i] = k_theta[i];
        }

        // Set joint impedance
        robot_->setJointImpedance(K_theta);

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set joint impedance", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::setCartesianImpedance(
    capnp::CallContext<SetCartesianImpedanceParams, SetCartesianImpedanceResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto k_x = params.getKX();

    if (k_x.size() != 6) {
        KJ_FAIL_REQUIRE("Cartesian impedance array must have exactly 6 elements");
    }

    try {
        // Convert to std::array
        std::array<double, 6> K_x{};
        for (size_t i = 0; i < 6; i++) {
            K_x[i] = k_x[i];
        }

        // Set Cartesian impedance
        robot_->setCartesianImpedance(K_x);

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set Cartesian impedance", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

