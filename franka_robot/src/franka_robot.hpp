#include <capnp/ez-rpc.h>
#include <capnp/message.h>
#include <capnp/serialize.h>
#include "rpc.capnp.h"

#include <iostream>
#include <array>
#include <vector>

class FrankaRobot {
public:
    FrankaRobot(const std::string& host, int port) 
        : client(host, port), rpcInterface(client.getMain<RPCService>()) {}

    void initializeRobot(const std::string& ip) {
        auto request = rpcInterface.initializeRobotRequest();
        request.setIpAddress(ip);
        try {
            request.send().wait(client.getWaitScope());
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    void initializeGripper() {
        auto request = rpcInterface.initializeGripperRequest();
        try {
            request.send().wait(client.getWaitScope());
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    void initializeVacuumGripper() {
        auto request = rpcInterface.initializeVacuumGripperRequest();
        try {
            request.send().wait(client.getWaitScope());
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    RobotState::Reader getRobotState() {
        auto request = rpcInterface.getRobotStateRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getState();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    void automaticErrorRecovery() {
        auto request = rpcInterface.automaticErrorRecoveryRequest();
        try {
            request.send().wait(client.getWaitScope());
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    std::vector<std::array<double, 16>> getJointPoses() {
        auto request = rpcInterface.getJointPosesRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            auto poses = response.getPoses();
            std::vector<std::array<double, 16>> result;
            result.reserve(poses.size());
            
            for (auto pose : poses) {
                std::array<double, 16> matrix;
                for (size_t i = 0; i < 16; i++) {
                    matrix[i] = pose[i];
                }
                result.push_back(matrix);
            }
            return result;
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool jointPointToPointMotion(const std::array<double, 7>& target_config, double speed_factor) {
        auto request = rpcInterface.jointPointToPointMotionRequest();
        auto target = request.initTargetConfiguration(7);
        for (size_t i = 0; i < 7; i++) {
            target.set(i, target_config[i]);
        }
        request.setSpeedFactor(speed_factor);

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getResult();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool jointTrajectoryMotion(const std::vector<std::array<double, 7>>& positions) {
        auto request = rpcInterface.jointTrajectoryMotionRequest();
        auto trajectory = request.initTrajectory(positions.size());

        for (size_t i = 0; i < positions.size(); i++) {
            auto point = trajectory[i];
            auto pos = point.initPositions(7);
            for (size_t j = 0; j < 7; j++) {
                pos.set(j, positions[i][j]);
            }
        }

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getResult();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    GripperState::Reader getGripperState() {
        auto request = rpcInterface.getGripperStateRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getState();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool gripperHoming() {
        auto request = rpcInterface.gripperHomingRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool gripperGrasp(double width, double speed, double force, 
                     double epsilon_inner, double epsilon_outer) {
        auto request = rpcInterface.gripperGraspRequest();
        request.setWidth(width);
        request.setSpeed(speed);
        request.setForce(force);
        request.setEpsilonInner(epsilon_inner);
        request.setEpsilonOuter(epsilon_outer);

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool gripperMove(double width, double speed) {
        auto request = rpcInterface.gripperMoveRequest();
        request.setWidth(width);
        request.setSpeed(speed);

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool gripperStop() {
        auto request = rpcInterface.gripperStopRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool setCollisionBehavior(
        const std::array<double, 7>& lower_torque_thresholds_acceleration,
        const std::array<double, 7>& upper_torque_thresholds_acceleration,
        const std::array<double, 7>& lower_torque_thresholds_nominal,
        const std::array<double, 7>& upper_torque_thresholds_nominal,
        const std::array<double, 6>& lower_force_thresholds_acceleration,
        const std::array<double, 6>& upper_force_thresholds_acceleration,
        const std::array<double, 6>& lower_force_thresholds_nominal,
        const std::array<double, 6>& upper_force_thresholds_nominal) {
        
        auto request = rpcInterface.setCollisionBehaviorRequest();
        
        auto lower_torque_acc = request.initLowerTorqueThresholdsAcceleration(7);
        auto upper_torque_acc = request.initUpperTorqueThresholdsAcceleration(7);
        auto lower_torque_nom = request.initLowerTorqueThresholdsNominal(7);
        auto upper_torque_nom = request.initUpperTorqueThresholdsNominal(7);
        auto lower_force_acc = request.initLowerForceThresholdsAcceleration(6);
        auto upper_force_acc = request.initUpperForceThresholdsAcceleration(6);
        auto lower_force_nom = request.initLowerForceThresholdsNominal(6);
        auto upper_force_nom = request.initUpperForceThresholdsNominal(6);

        for (size_t i = 0; i < 7; i++) {
            lower_torque_acc.set(i, lower_torque_thresholds_acceleration[i]);
            upper_torque_acc.set(i, upper_torque_thresholds_acceleration[i]);
            lower_torque_nom.set(i, lower_torque_thresholds_nominal[i]);
            upper_torque_nom.set(i, upper_torque_thresholds_nominal[i]);
        }

        for (size_t i = 0; i < 6; i++) {
            lower_force_acc.set(i, lower_force_thresholds_acceleration[i]);
            upper_force_acc.set(i, upper_force_thresholds_acceleration[i]);
            lower_force_nom.set(i, lower_force_thresholds_nominal[i]);
            upper_force_nom.set(i, upper_force_thresholds_nominal[i]);
        }

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool setLoadInertia(
        double mass,
        const std::array<double, 3>& center_of_mass,
        const std::array<double, 9>& load_inertia) {
        auto request = rpcInterface.setLoadInertiaRequest();
        request.setMass(mass);
        auto center_of_mass_capnp = request.initCenterOfMass(3);
        for (size_t i = 0; i < 3; i++) {
            center_of_mass_capnp.set(i, center_of_mass[i]);
        }
        auto load_inertia_capnp = request.initLoadInertia(9);
        for (size_t i = 0; i < 9; i++) {
            load_inertia_capnp.set(i, load_inertia[i]);
        }

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    VacuumGripperState::Reader getVacuumGripperState() {
        auto request = rpcInterface.getVacuumGripperStateRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getState();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool vacuumGripperVacuum(uint8_t control_point, uint32_t timeout, uint8_t profile) {
        auto request = rpcInterface.vacuumGripperVacuumRequest();
        request.setControlPoint(control_point);
        request.setTimeout(timeout);
        request.setProfile(profile);

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool vacuumGripperDropOff(uint32_t timeout) {
        auto request = rpcInterface.vacuumGripperDropOffRequest();
        request.setTimeout(timeout);

        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

    bool vacuumGripperStop() {
        auto request = rpcInterface.vacuumGripperStopRequest();
        try {
            auto response = request.send().wait(client.getWaitScope());
            return response.getSuccess();
        } catch (const kj::Exception& e) {
            std::cerr << "Franka Robot Error: " << e.getDescription().cStr() << std::endl;
            throw;
        }
    }

private:
    capnp::EzRpcClient client;
    RPCService::Client rpcInterface;
};