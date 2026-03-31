classdef FrankaGripperControlModes < Simulink.IntEnumType
    %  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    enumeration
        idle (0)
        read_state (1)
        homing (2)
        grasp (3)
        move (4)
        stop (5)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = FrankaGripperControlModes.idle;
        end
    end
end