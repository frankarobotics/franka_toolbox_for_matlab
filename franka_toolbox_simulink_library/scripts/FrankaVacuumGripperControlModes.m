classdef FrankaVacuumGripperControlModes < Simulink.IntEnumType
    %  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    enumeration
        idle (0)
        read_state (1)
        vacuum (2)
        dropOff (3)
        stop (4)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = FrankaVacuumGripperControlModes.idle;
        end
    end
end