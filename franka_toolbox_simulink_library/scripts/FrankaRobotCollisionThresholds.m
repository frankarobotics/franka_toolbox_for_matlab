classdef FrankaRobotCollisionThresholds
    %  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    properties
        lower_torque_thresholds_acceleration = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds_acceleration = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    end
    
    methods
        function collision_thresholds = getAs1DArray(obj)
            collision_thresholds = [obj.lower_torque_thresholds_acceleration,...
                                    obj.upper_torque_thresholds_acceleration,...
                                    obj.lower_torque_thresholds_nominal,...
                                    obj.upper_torque_thresholds_nominal,...
                                    obj.lower_force_thresholds_acceleration,...
                                    obj.upper_force_thresholds_acceleration,...
                                    obj.lower_force_thresholds_nominal,...
                                    obj.upper_force_thresholds_nominal];
        end
    end
end