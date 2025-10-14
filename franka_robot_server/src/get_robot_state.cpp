#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <sstream>

kj::Promise<void> FrankaRobotRPCServiceImpl::getRobotState(
    capnp::CallContext<GetRobotStateParams, GetRobotStateResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto results = context.getResults();
    auto state = results.initState();

    robot_->read([&](const franka::RobotState& robot_state) {
        // Transform matrices (4x4)
        {
            auto oTEe = state.initOTEe(16);
            auto oTEeD = state.initOTEeD(16);
            auto fTEe = state.initFTEe(16);
            auto fTNe = state.initFTNe(16);
            auto neTEe = state.initNeTEe(16);
            auto eeTK = state.initEeTK(16);
            auto oTEeC = state.initOTEeC(16);

            for (size_t i = 0; i < 16; ++i) {
                oTEe.set(i, robot_state.O_T_EE[i]);
                oTEeD.set(i, robot_state.O_T_EE_d[i]);
                fTEe.set(i, robot_state.F_T_EE[i]);
                fTNe.set(i, robot_state.F_T_NE[i]);
                neTEe.set(i, robot_state.NE_T_EE[i]);
                eeTK.set(i, robot_state.EE_T_K[i]);
                oTEeC.set(i, robot_state.O_T_EE_c[i]);
            }
        }

        // Mass and inertia properties
        state.setMEe(robot_state.m_ee);
        state.setMLoad(robot_state.m_load);
        state.setMTotal(robot_state.m_total);

        {
            auto iEe = state.initIEe(9);
            auto iLoad = state.initILoad(9);
            auto iTotal = state.initITotal(9);
            
            for (size_t i = 0; i < 9; ++i) {
                iEe.set(i, robot_state.I_ee[i]);
                iLoad.set(i, robot_state.I_load[i]);
                iTotal.set(i, robot_state.I_total[i]);
            }
        }

        // Center of mass positions
        {
            auto fXCee = state.initFXCee(3);
            auto fXCload = state.initFXCload(3);
            auto fXCtotal = state.initFXCtotal(3);
            
            for (size_t i = 0; i < 3; ++i) {
                fXCee.set(i, robot_state.F_x_Cee[i]);
                fXCload.set(i, robot_state.F_x_Cload[i]);
                fXCtotal.set(i, robot_state.F_x_Ctotal[i]);
            }
        }

        // Elbow configuration
        {
            auto elbow = state.initElbow(2);
            auto elbowD = state.initElbowD(2);
            auto elbowC = state.initElbowC(2);
            auto delbowC = state.initDelbowC(2);
            auto ddelbowC = state.initDdelbowC(2);
            
            for (size_t i = 0; i < 2; ++i) {
                elbow.set(i, robot_state.elbow[i]);
                elbowD.set(i, robot_state.elbow_d[i]);
                elbowC.set(i, robot_state.elbow_c[i]);
                delbowC.set(i, robot_state.delbow_c[i]);
                ddelbowC.set(i, robot_state.ddelbow_c[i]);
            }
        }

        // Joint states (7-DOF)
        {
            auto tauJ = state.initTauJ(7);
            auto tauJD = state.initTauJD(7);
            auto dtauJ = state.initDtauJ(7);
            auto q = state.initQ(7);
            auto qD = state.initQD(7);
            auto dq = state.initDq(7);
            auto dqD = state.initDqD(7);
            auto ddqD = state.initDdqD(7);
            auto theta = state.initTheta(7);
            auto dtheta = state.initDtheta(7);
            auto tauExtHatFiltered = state.initTauExtHatFiltered(7);
            
            for (size_t i = 0; i < 7; ++i) {
                tauJ.set(i, robot_state.tau_J[i]);
                tauJD.set(i, robot_state.tau_J_d[i]);
                dtauJ.set(i, robot_state.dtau_J[i]);
                q.set(i, robot_state.q[i]);
                qD.set(i, robot_state.q_d[i]);
                dq.set(i, robot_state.dq[i]);
                dqD.set(i, robot_state.dq_d[i]);
                ddqD.set(i, robot_state.ddq_d[i]);
                theta.set(i, robot_state.theta[i]);
                dtheta.set(i, robot_state.dtheta[i]);
                tauExtHatFiltered.set(i, robot_state.tau_ext_hat_filtered[i]);
            }
        }

        // Contact and collision states
        {
            auto jointContact = state.initJointContact(7);
            auto jointCollision = state.initJointCollision(7);
            
            for (size_t i = 0; i < 7; ++i) {
                jointContact.set(i, robot_state.joint_contact[i]);
                jointCollision.set(i, robot_state.joint_collision[i]);
            }
        }

        {
            auto cartesianContact = state.initCartesianContact(6);
            auto cartesianCollision = state.initCartesianCollision(6);
            auto oFExtHatK = state.initOFExtHatK(6);
            auto kFExtHatK = state.initKFExtHatK(6);
            auto oDpEeD = state.initODpEeD(6);
            auto oDpEeC = state.initODpEeC(6);
            auto oDdpEeC = state.initODdpEeC(6);
            
            for (size_t i = 0; i < 6; ++i) {
                cartesianContact.set(i, robot_state.cartesian_contact[i]);
                cartesianCollision.set(i, robot_state.cartesian_collision[i]);
                oFExtHatK.set(i, robot_state.O_F_ext_hat_K[i]);
                kFExtHatK.set(i, robot_state.K_F_ext_hat_K[i]);
                oDpEeD.set(i, robot_state.O_dP_EE_d[i]);
                oDpEeC.set(i, robot_state.O_dP_EE_c[i]);
                oDdpEeC.set(i, robot_state.O_ddP_EE_c[i]);
            }
        }

        // Error states and control command success rate
        std::stringstream current_errors_ss;
        current_errors_ss << robot_state.current_errors;
        state.setCurrentErrors(current_errors_ss.str());

        std::stringstream last_motion_errors_ss;
        last_motion_errors_ss << robot_state.last_motion_errors;
        state.setLastMotionErrors(last_motion_errors_ss.str());

        state.setControlCommandSuccessRate(robot_state.control_command_success_rate);

        return false;  // Stop reading after one state
    });

    return kj::READY_NOW;
} 