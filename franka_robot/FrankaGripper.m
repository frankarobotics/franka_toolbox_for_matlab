classdef FrankaGripper < handle
    
    properties (Access = private)
        frankaRobotHandle
        isInitialized = false
    end
    
    methods
        %% Constructor
        function obj = FrankaGripper(frankaRobotHandle)
            % FrankaGripper constructor
            % Input:
            %   frankaRobotHandle - Handle to the Franka robot instance
            obj.frankaRobotHandle = frankaRobotHandle;
        end
        
        %% Private Methods
        function initializeGripper(obj)
            % Initialize the gripper on the robot
            if ~isempty(obj.frankaRobotHandle) && ~obj.isInitialized
                franka_robot('initialize_gripper', obj.frankaRobotHandle);
            end
        end
        
        %% Gripper State
        function state = state(obj)
            % Get the current state of the gripper
            % Returns:
            %   state - Current gripper state
            obj.initializeGripper();
            state = franka_robot('gripper_state', obj.frankaRobotHandle);
        end
        
        %% Gripper Homing
        function result = homing(obj)
            % Perform gripper homing
            % Returns:
            %   result - True if homing was successful
            obj.initializeGripper();
            result = franka_robot('gripper_homing', obj.frankaRobotHandle);
        end
        
        %% Gripper Grasp
        function result = grasp(obj, width, speed, force, epsilon_inner, epsilon_outer)
            % Grasp an object with the gripper
            % Inputs:
            %   width - Target width in meters
            %   speed - Speed of the motion (default: 0.1)
            %   force - Grasping force in N (default: 50)
            %   epsilon_inner - Inner epsilon for grasping (default: 0.1)
            %   epsilon_outer - Outer epsilon for grasping (default: 0.1)
            % Returns:
            %   result - True if grasping was successful
            obj.initializeGripper();
            % Set default values if not provided
            if nargin < 6
                epsilon_outer = 0.1;
            end
            if nargin < 5
                epsilon_inner = 0.1;
            end
            if nargin < 4
                force = 50;
            end
            if nargin < 3
                speed = 0.1;
            end
            
            result = franka_robot('gripper_grasp', obj.frankaRobotHandle, ...
                width, speed, force, epsilon_inner, epsilon_outer);
        end
        
        %% Gripper Move
        function result = move(obj, width, speed)
            % Move the gripper to a specific width
            % Inputs:
            %   width - Target width in meters
            %   speed - Speed of the motion (default: 0.1)
            % Returns:
            %   result - True if motion was successful
            obj.initializeGripper();
            if nargin < 3
                speed = 0.1; % Default speed
            end
            result = franka_robot('gripper_move', obj.frankaRobotHandle, width, speed);
        end
        
        %% Gripper Stop
        function result = stop(obj)
            % Stop the gripper motion
            % Returns:
            %   result - True if stop was successful
            obj.initializeGripper();
            result = franka_robot('gripper_stop', obj.frankaRobotHandle);
        end
    end
end 