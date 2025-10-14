classdef FrankaVacuumGripper < handle
    
    properties (Access = private)
        frankaRobotHandle
        isInitialized = false
    end
    
    properties (Constant, Access = private)
        % Default values
        DefaultControlPoint = 0;
        DefaultTimeout = 5000;  % 5 seconds in milliseconds
        DefaultProfile = 0;     % Default production setup profile
    end
    
    methods
        %% Constructor
        function obj = FrankaVacuumGripper(frankaRobotHandle)
            % FrankaVacuumGripper constructor
            % Input:
            %   frankaRobotHandle - Handle to the Franka robot instance
            obj.frankaRobotHandle = frankaRobotHandle;
        end
        
        %% Private Methods
        function initializeVacuumGripper(obj)
            % Initialize the vacuum gripper on the robot
            if ~isempty(obj.frankaRobotHandle) && ~obj.isInitialized
                franka_robot('initialize_vacuum_gripper', obj.frankaRobotHandle);
                obj.isInitialized = true;
            end
        end
        
        %% Vacuum Gripper State
        function state = state(obj)
            % Get the current state of the vacuum gripper
            % Returns:
            %   state - Structure containing:
            %       in_control_range - Whether the vacuum gripper is in control range
            %       part_detached - Whether a part is detached
            %       part_present - Whether a part is present
            %       device_status - Current device status
            %       actual_power - Current actual power
            %       vacuum - Current system vacuum
            %       time - Time stamp
            obj.initializeVacuumGripper();
            state = franka_robot('vacuum_gripper_state', obj.frankaRobotHandle);
        end
        
        %% Vacuum Gripper Vacuum
        function result = vacuum(obj, control_point, timeout, profile)
            % Apply vacuum to the gripper
            % Inputs:
            %   control_point - Vacuum control point (default: 0)
            %   timeout - Timeout in milliseconds (default: 5000)
            %   profile - Production setup profile (default: 0)
            % Returns:
            %   result - True if vacuum was successful
            obj.initializeVacuumGripper();
            % Set default values if not provided
            if nargin < 4
                profile = obj.DefaultProfile;
            end
            if nargin < 3
                timeout = obj.DefaultTimeout;
            end
            if nargin < 2
                control_point = obj.DefaultControlPoint;
            end
            
            result = franka_robot('vacuum_gripper_vacuum', obj.frankaRobotHandle, ...
                control_point, timeout, profile);
        end
        
        %% Vacuum Gripper Drop Off
        function result = dropOff(obj, timeout)
            % Drop off the currently held object
            % Inputs:
            %   timeout - Timeout in milliseconds (default: 5000)
            % Returns:
            %   result - True if drop off was successful
            obj.initializeVacuumGripper();
            if nargin < 2
                timeout = obj.DefaultTimeout;
            end
            
            result = franka_robot('vacuum_gripper_drop_off', obj.frankaRobotHandle, timeout);
        end
        
        %% Vacuum Gripper Stop
        function result = stop(obj)
            % Stop the vacuum gripper
            % Returns:
            %   result - True if stop was successful
            obj.initializeVacuumGripper();
            result = franka_robot('vacuum_gripper_stop', obj.frankaRobotHandle);
        end
        
    end
end 