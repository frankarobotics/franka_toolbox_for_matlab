#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <chrono>

kj::Promise<void> FrankaRobotRPCServiceImpl::ping(
    capnp::CallContext<PingParams, PingResults> context) {
    
    auto results = context.getResults();
    
    // Get current timestamp in milliseconds since epoch
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    
    results.setTimestamp(static_cast<uint64_t>(millis));
    results.setPort(server_port_);
    
    return kj::READY_NOW;
}

