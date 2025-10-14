// Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSE' , which is part of this package

#include "local_utils.h"

// Replace all occurrences of a substring in a string
void replaceStringInPlace(std::string& subject, const std::string& search, const std::string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
}

// Convert a space-separated string of signal names into a vector of strings
void signalsStringToSignalsVector(std::vector<std::string>& signals, const char* signalsString) {
    std::string outputSignalsString = signalsString;

    signals.clear();
    int outputSignalsCounter = 0;
    int localCounter = 0;

    for (int i = 0; i < outputSignalsString.length(); i++) {
        if (outputSignalsString[i] == ' ') {
            signals.push_back(outputSignalsString.substr(i - localCounter, localCounter));
            outputSignalsCounter++;
            localCounter = 0;
        } else {
            localCounter++;
        }
    }
    signals.push_back(
        outputSignalsString.substr(outputSignalsString.length() - localCounter, localCounter));
}

// Map of signal names to their widths
std::map<std::string, int> getWidthForSignalString = {
    {"O_T_EE", 16},
    {"O_T_EE_d", 16},
    {"F_T_EE", 16},
    {"EE_T_K", 16},
    {"m_ee", 1},
    {"I_ee", 9},
    {"F_x_Cee", 3},
    {"m_load", 1},
    {"I_load", 9},
    {"F_x_Cload", 3},
    {"m_total", 1},
    {"I_total", 9},
    {"F_x_Ctotal", 3},
    {"elbow", 2},
    {"elbow_d", 2},
    {"elbow_c", 2},
    {"delbow_c", 2},
    {"ddelbow_c", 2},
    {"tau_J", 7},
    {"tau_J_d", 7},
    {"dtau_J", 7},
    {"q", 7},
    {"q_d", 7},
    {"dq", 7},
    {"dq_d", 7},
    {"ddq_d", 7},
    {"joint_contact", 7},
    {"cartesian_contact", 6},
    {"joint_collision", 7},
    {"cartesian_collision", 6},
    {"tau_ext_hat_filtered", 7},
    {"O_F_ext_hat_K", 6},
    {"K_F_ext_hat_K", 6},
    {"O_dP_EE_d", 6},
    {"O_T_EE_c", 16},
    {"O_dP_EE_c", 6},
    {"O_ddP_EE_c", 6},
    {"theta", 7},
    {"dtheta", 7},
    {"control_command_success_rate", 1},
    {"robot_mode", 1},
    {"time", 1}
}; 