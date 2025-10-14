#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <memory>
#include <chrono>

kj::Promise<void> FrankaRobotRPCServiceImpl::getVacuumGripperState(
    capnp::CallContext<GetVacuumGripperStateParams, GetVacuumGripperStateResults> context) {
    try {
        auto gripper_state = vacuum_gripper_->readOnce();
        auto state = context.getResults().getState();
        
        state.setInControlRange(gripper_state.in_control_range);
        state.setPartDetached(gripper_state.part_detached);
        state.setPartPresent(gripper_state.part_present);
        state.setDeviceStatus(static_cast<uint8_t>(gripper_state.device_status));  // Cast enum to uint8_t
        state.setActualPower(static_cast<double>(gripper_state.actual_power));
        state.setVacuum(static_cast<double>(gripper_state.vacuum));
        state.setTime(static_cast<double>(gripper_state.time.toMSec()));
        
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        return kj::Promise<void>(kj::Exception(kj::Exception::Type::FAILED, __FILE__, __LINE__,
                                             kj::str("Failed to get vacuum gripper state: ", e.what())));
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperVacuum(
    capnp::CallContext<VacuumGripperVacuumParams, VacuumGripperVacuumResults> context) {
    try {
        auto params = context.getParams();
        bool success = vacuum_gripper_->vacuum(
            params.getControlPoint(),
            std::chrono::milliseconds(params.getTimeout()),  // Convert to milliseconds
            static_cast<franka::VacuumGripper::ProductionSetupProfile>(params.getProfile())  // Cast to enum
        );
        context.getResults().setSuccess(success);
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        return kj::Promise<void>(kj::Exception(kj::Exception::Type::FAILED, __FILE__, __LINE__,
                                             kj::str("Failed to execute vacuum: ", e.what())));
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperDropOff(
    capnp::CallContext<VacuumGripperDropOffParams, VacuumGripperDropOffResults> context) {
    try {
        auto params = context.getParams();
        bool success = vacuum_gripper_->dropOff(std::chrono::milliseconds(params.getTimeout()));  // Convert to milliseconds
        context.getResults().setSuccess(success);
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        return kj::Promise<void>(kj::Exception(kj::Exception::Type::FAILED, __FILE__, __LINE__,
                                             kj::str("Failed to execute drop off: ", e.what())));
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperStop(
    capnp::CallContext<VacuumGripperStopParams, VacuumGripperStopResults> context) {
    try {
        bool success = vacuum_gripper_->stop();
        context.getResults().setSuccess(success);
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        return kj::Promise<void>(kj::Exception(kj::Exception::Type::FAILED, __FILE__, __LINE__,
                                             kj::str("Failed to stop vacuum gripper: ", e.what())));
    }
} 