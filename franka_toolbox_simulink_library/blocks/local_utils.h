// Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSE' , which is part of this package

#ifndef LOCAL_UTILS_H
#define LOCAL_UTILS_H

#include <string>
#include <vector>
#include <map>

// Replace all occurrences of a substring in a string
void replaceStringInPlace(std::string& subject, const std::string& search, const std::string& replace);

// Convert a space-separated string of signal names into a vector of strings
void signalsStringToSignalsVector(std::vector<std::string>& signals, const char* signalsString);

// Map of signal names to their widths
extern std::map<std::string, int> getWidthForSignalString;

#endif // LOCAL_UTILS_H 