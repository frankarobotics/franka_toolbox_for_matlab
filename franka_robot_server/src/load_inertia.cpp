#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <array>

kj::Promise<void> FrankaRobotRPCServiceImpl::setLoadInertia(
    capnp::CallContext<SetLoadInertiaParams, SetLoadInertiaResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    
    // Get parameters
    double mass = params.getMass();
    auto center_of_mass = params.getCenterOfMass();
    auto load_inertia = params.getLoadInertia();

    // Validate array sizes
    if (center_of_mass.size() != 3) {
        KJ_FAIL_REQUIRE("Center of mass must have exactly 3 elements");
    }
    if (load_inertia.size() != 9) {
        KJ_FAIL_REQUIRE("Load inertia must have exactly 9 elements (3x3 matrix in row-major format)");
    }

    try {
        // Convert to std::array
        std::array<double, 3> com{};
        std::array<double, 9> inertia{};

        for (size_t i = 0; i < 3; i++) {
            com[i] = center_of_mass[i];
        }

        for (size_t i = 0; i < 9; i++) {
            inertia[i] = load_inertia[i];
        }

        // Set load parameters
        robot_->setLoad(mass, com, inertia);

        auto results = context.getResults();
        results.setSuccess(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to set load inertia", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
} 