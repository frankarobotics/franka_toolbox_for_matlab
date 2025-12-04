classdef FrankaRobotLoadInertia
    %  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    properties
        mass = 0
        center_of_mass = [0,0,0]
        inertia_matrix =  [0.001,0,0;...
                           0,0.001,0;...
                           0,0,0.001]
    end
    methods
        function load_inertia = getAs1DArray(obj)
            load_inertia = [obj.mass, obj.center_of_mass, reshape(obj.inertia_matrix,1,9)];
        end
    end
end