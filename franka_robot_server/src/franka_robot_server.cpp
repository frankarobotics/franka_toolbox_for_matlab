#include <capnp/ez-rpc.h>
#include <capnp/message.h>
#include <capnp/serialize.h>
#include "rpc.capnp.h"

#include <iostream>
#include "franka_robot_server/franka_robot_rpc_service.hpp"

int main(int argc, char* argv[]) {
    // Check if the correct number of arguments is provided
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <IP_ADDRESS> <PORT>\n";
        return 1;
    }

    // Parse IP address and port from the command-line arguments
    std::string ipAddress = argv[1];
    int port = std::stoi(argv[2]);

    // Initialize the RPC server with the provided IP address and port
    capnp::EzRpcServer server(kj::heap<FrankaRobotRPCServiceImpl>(), ipAddress, port);
    
    auto& waitScope = server.getWaitScope();

    // Run forever, accepting connections and handling requests.
    kj::NEVER_DONE.wait(waitScope);

    return 0;
} 