#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <array>
#include <iostream>

kj::Promise<void> FrankaRobotRPCServiceImpl::getJointPoses(
    capnp::CallContext<GetJointPosesParams, GetJointPosesResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        // Create model if not already created
        if (!model_) {
            try {
                model_ = std::make_unique<franka::Model>(robot_->loadModel());
            } catch (const franka::Exception& e) {
                KJ_FAIL_REQUIRE("Failed to load robot model", e.what());
            }
        }

        auto results = context.getResults();
        const size_t num_frames = 10;  // Match the client expectation
        
        // Pre-allocate all poses before starting the robot read
        std::array<std::array<double, 16>, 10> frame_poses{};
        bool success = false;

        try {
            robot_->read([&](const franka::RobotState& robot_state) {
                if (robot_state.q.empty()) {
                    return false;  // Will throw appropriate error after read
                }

                // Calculate poses for valid frames only
                for (size_t i = 0; i < num_frames && i < 10; i++) {
                    try {
                        frame_poses[i] = model_->pose(franka::Frame(i), robot_state);
                    } catch (const franka::Exception& e) {
                        // Log error but continue with remaining frames
                        std::cerr << "Warning: Failed to calculate pose for frame " << i 
                                << ": " << e.what() << std::endl;
                        // Fill with identity matrix
                        frame_poses[i].fill(0.0);
                        for (size_t j = 0; j < 4; j++) {
                            frame_poses[i][j * 4 + j] = 1.0;
                        }
                    }
                }
                
                success = true;
                return false;
            });
        } catch (const franka::Exception& e) {
            KJ_FAIL_REQUIRE("Failed to read robot state", e.what());
        }

        if (!success) {
            KJ_FAIL_REQUIRE("Failed to read robot state: empty joint positions");
        }

        // Initialize output poses with correct dimensions
        auto poses = results.initPoses(num_frames);
        
        // Copy poses to output with proper initialization of inner lists
        for (size_t i = 0; i < num_frames; i++) {
            auto poseList = poses.init(i, 16);  // Properly initialize inner list with size
            for (size_t j = 0; j < 16; j++) {
                poseList.set(j, frame_poses[i][j]);
            }
        }

    } catch (const franka::Exception& e) {
        KJ_FAIL_REQUIRE("Franka exception occurred", e.what());
    } catch (const std::exception& e) {
        KJ_FAIL_REQUIRE("Standard exception occurred", e.what());
    } catch (...) {
        KJ_FAIL_REQUIRE("Unknown error occurred");
    }

    return kj::READY_NOW;
} 