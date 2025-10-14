#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <array>

kj::Promise<void> FrankaRobotRPCServiceImpl::setCollisionBehavior(
    capnp::CallContext<SetCollisionBehaviorParams, SetCollisionBehaviorResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    
    // Get collision thresholds from parameters
    auto lower_torque_thresholds_acc = params.getLowerTorqueThresholdsAcceleration();
    auto upper_torque_thresholds_acc = params.getUpperTorqueThresholdsAcceleration();
    auto lower_torque_thresholds_nom = params.getLowerTorqueThresholdsNominal();
    auto upper_torque_thresholds_nom = params.getUpperTorqueThresholdsNominal();
    auto lower_force_thresholds_acc = params.getLowerForceThresholdsAcceleration();
    auto upper_force_thresholds_acc = params.getUpperForceThresholdsAcceleration();
    auto lower_force_thresholds_nom = params.getLowerForceThresholdsNominal();
    auto upper_force_thresholds_nom = params.getUpperForceThresholdsNominal();

    // Validate array sizes
    if (lower_torque_thresholds_acc.size() != 7 || upper_torque_thresholds_acc.size() != 7 ||
        lower_torque_thresholds_nom.size() != 7 || upper_torque_thresholds_nom.size() != 7) {
        KJ_FAIL_REQUIRE("Torque threshold arrays must have exactly 7 elements");
    }

    if (lower_force_thresholds_acc.size() != 6 || upper_force_thresholds_acc.size() != 6 ||
        lower_force_thresholds_nom.size() != 6 || upper_force_thresholds_nom.size() != 6) {
        KJ_FAIL_REQUIRE("Force threshold arrays must have exactly 6 elements");
    }

    try {
        // Convert to std::array
        std::array<double, 7> lower_torque_acc{};
        std::array<double, 7> upper_torque_acc{};
        std::array<double, 7> lower_torque_nom{};
        std::array<double, 7> upper_torque_nom{};
        std::array<double, 6> lower_force_acc{};
        std::array<double, 6> upper_force_acc{};
        std::array<double, 6> lower_force_nom{};
        std::array<double, 6> upper_force_nom{};

        for (size_t i = 0; i < 7; i++) {
            lower_torque_acc[i] = lower_torque_thresholds_acc[i];
            upper_torque_acc[i] = upper_torque_thresholds_acc[i];
            lower_torque_nom[i] = lower_torque_thresholds_nom[i];
            upper_torque_nom[i] = upper_torque_thresholds_nom[i];
        }

        for (size_t i = 0; i < 6; i++) {
            lower_force_acc[i] = lower_force_thresholds_acc[i];
            upper_force_acc[i] = upper_force_thresholds_acc[i];
            lower_force_nom[i] = lower_force_thresholds_nom[i];
            upper_force_nom[i] = upper_force_thresholds_nom[i];
        }

        // Set collision behavior
        robot_->setCollisionBehavior(
            lower_torque_acc,
            upper_torque_acc,
            lower_torque_nom,
            upper_torque_nom,
            lower_force_acc,
            upper_force_acc,
            lower_force_nom,
            upper_force_nom
        );

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set collision behavior", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
} 