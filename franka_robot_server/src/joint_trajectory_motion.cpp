#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/control_types.h>
#include <array>
#include <vector>
#include <cmath>
#include <sstream>

namespace {
    static constexpr double kDeltaQMotionFinished = 1e-2;

    // Helper function to create error message for invalid initial position
    std::string createErrorMessage(const std::array<double, 7>& target_position, 
                                 const std::array<double, 7>& current_position) {
        std::ostringstream os;
        os << "The first trajectory point [";
        for (size_t i = 0; i < 7; ++i) {
            os << target_position[i] << (i < 6 ? ", " : "");
        }
        os << "]\nshould match the current robot joint configuration\n[";
        for (size_t i = 0; i < 7; ++i) {
            os << current_position[i] << (i < 6 ? ", " : "");
        }
        os << "]\n";
        return os.str();
    }

    // Helper function to check if initial position is valid
    bool isInitialPositionValid(const std::array<double, 7>& current_position, 
                              const std::array<double, 7>& target_position) {
        for (size_t i = 0; i < 7; ++i) {
            if (std::abs(current_position[i] - target_position[i]) > kDeltaQMotionFinished) {
                return false;
            }
        }
        return true;
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::jointTrajectoryMotion(
    capnp::CallContext<JointTrajectoryMotionParams, JointTrajectoryMotionResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto trajectory = params.getTrajectory();

    // Validate trajectory
    if (trajectory.size() == 0) {
        KJ_FAIL_REQUIRE("Trajectory cannot be empty");
    }

    // Convert trajectory points to vectors
    std::vector<std::array<double, 7>> positions;

    for (auto point : trajectory) {
        // Validate point data
        if (point.getPositions().size() != 7) {
            KJ_FAIL_REQUIRE("Each trajectory point must have 7 position values");
        }

        // Convert positions
        std::array<double, 7> pos{};
        auto pos_list = point.getPositions();
        for (size_t i = 0; i < 7; ++i) {
            pos[i] = pos_list[i];
        }
        positions.push_back(pos);
    }

    try {
        // Check if initial position matches current robot position
        auto current_state = robot_->readOnce();
        if (!isInitialPositionValid(current_state.q, positions[0])) {
            std::string error_msg = createErrorMessage(positions[0], current_state.q);
            KJ_FAIL_REQUIRE(error_msg.c_str());
        }

        // Create trajectory generator and execute motion
        uint64_t time_ms = 0;
        franka::JointPositions output = {{0, 0, 0, 0, 0, 0, 0}};
        
        auto control_callback = [&positions, &time_ms, &output](
            const franka::RobotState& /*state*/,
            franka::Duration period) -> franka::JointPositions {
            
            time_ms += period.toMSec();
            size_t current_point = std::min(static_cast<size_t>(time_ms), positions.size() - 1);
            
            output = franka::JointPositions(positions[current_point]);

            if (current_point >= positions.size() - 1) {
                return franka::MotionFinished(output);
            }
            
            return output;
        };

        robot_->control(control_callback, franka::ControllerMode::kJointImpedance, true);

        // Set success result
        auto results = context.getResults();
        results.setResult(true);

    } catch (const franka::Exception& e) {
        auto results = context.getResults();
        results.setResult(false);
        KJ_LOG(ERROR, "Joint trajectory motion failed", e.what());
    }

    return kj::READY_NOW;
} 