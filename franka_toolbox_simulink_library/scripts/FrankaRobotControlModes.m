classdef FrankaRobotControlModes  < Simulink.IntEnumType
    %  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    enumeration
      torqueControl (1)
      torqueControlJointPositions (2)
      torqueControlJointVelocities (3)
      torqueControlCartesianPose (4)
      torqueControlCartesianVelocities (5)
      jointPositionsControllerMode (6)
      jointVelocitiesControllerMode (7)
      cartesianPoseControllerMode (8)
      cartesianVelocitiesControllerMode (9)
      torqueControlCartesianPoseWithElbow (10)
      torqueControlCartesianVelocitiesWithElbow (11)
      cartesianPoseControllerModeWithElbow (12)
      cartesianVelocitiesControllerModeWithElbow (13)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = FrankaRobotControlModes.torqueControl;
        end
    end
end