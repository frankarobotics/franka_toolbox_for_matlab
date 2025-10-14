classdef FrankaRobot < handle
    
    properties (Constant, Access = private)
        % Set standard default values for the Remote Server
        DefaultServerUsername = 'franka';
        DefaultServerIP = '172.16.1.2';
        DefaultSSHPort = '22';
        DefaultServerPort = '5001';

        % Set standard default values for the Robot Settings
        DefaultTorqueThresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0];
        
        DefaultForceThresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0];

        DefaultCollisionThresholds = struct(...
            'lower_torque_thresholds_acceleration', FrankaRobot.DefaultTorqueThresholds, ...
            'upper_torque_thresholds_acceleration', FrankaRobot.DefaultTorqueThresholds, ...
            'lower_torque_thresholds_nominal', FrankaRobot.DefaultTorqueThresholds, ...
            'upper_torque_thresholds_nominal', FrankaRobot.DefaultTorqueThresholds, ...
            'lower_force_thresholds_acceleration', FrankaRobot.DefaultForceThresholds, ...
            'upper_force_thresholds_acceleration', FrankaRobot.DefaultForceThresholds, ...
            'lower_force_thresholds_nominal', FrankaRobot.DefaultForceThresholds, ...
            'upper_force_thresholds_nominal', FrankaRobot.DefaultForceThresholds ...
        )
        
        DefaultLoadInertia = struct(...
            'mass', 0, ...
            'center_of_mass', [0,0,0], ...
            'load_inertia', [0.001,0,0;0,0.001,0;0,0,0.001] ...
        )
        % Robot IP Address
        DefaultRobotIP = '172.16.0.2';

        % Default Settings
        DefaultSettings = struct(...
            'homeConfiguration', [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4], ...
            'collisionThresholds', FrankaRobot.DefaultCollisionThresholds, ...
            'loadInertia', FrankaRobot.DefaultLoadInertia ...
        )
    end
    
    properties
        Settings
        Server
        Gripper
        VacuumGripper
    end
    
    properties (SetAccess = private, Hidden = true)
        RobotIP
        frankaRobotHandle
    end
    
    methods
        %% Constructor
        function obj = FrankaRobot(varargin)
            % Initialize Settings with DefaultSettings
            obj.Settings = obj.DefaultSettings;
            
            % Create input parser
            p = inputParser;
            
            % Define parameters with default values
            addParameter(p, 'RobotIP', FrankaRobot.DefaultRobotIP, @ischar);

            addParameter(p, 'Username', obj.DefaultServerUsername, @ischar);
            addParameter(p, 'ServerIP', obj.DefaultServerIP, @ischar);
            addParameter(p, 'SSHPort', obj.DefaultSSHPort, @ischar);
            addParameter(p, 'ServerPort', obj.DefaultServerPort, @ischar);
            
            % Parse the inputs
            parse(p, varargin{:});
            
            % Get the results
            params = p.Results;
            
            % Store the RobotIP in Settings
            obj.RobotIP = params.RobotIP;
            
            % Initialize server based on parameters
            if all(strcmp({params.Username, params.ServerIP, params.SSHPort, params.ServerPort}, ...
                    {obj.DefaultServerUsername, obj.DefaultServerIP, obj.DefaultSSHPort, obj.DefaultServerPort}))
                % Case 1: All parameters are default values - use local server
                obj.Server = FrankaRobotServer();
            else
                % Case 2: At least one server parameter was specified - use remote server
                obj.Server = FrankaRobotServer(params.Username, params.ServerIP, ...
                    params.SSHPort, params.ServerPort);
            end

            try
                obj.Server.start();
            catch ME
                ME = MException('FrankaRobot:InitError', 'Failed to start Franka Robot Server: %s', ME.message);
                throw(ME);
            end

            % Create a new Franka Robot handle for the client
            obj.frankaRobotHandle = franka_robot('new', obj.Server.getServerIp(), obj.Server.getServerPort());
            
            % Initialize the robot
            try
                obj.initialize();
            catch ME
                % Clean up the server and handle before throwing error
                if ~isempty(obj.Server)
                    obj.Server.stop();
                end
                if ~isempty(obj.frankaRobotHandle)
                    franka_robot('delete', obj.frankaRobotHandle);
                end
                ME = MException('FrankaRobot:InitError', 'Failed to initialize robot: %s', ME.message);
                throw(ME);
            end

            % Create the gripper instance
            obj.Gripper = FrankaGripper(obj.frankaRobotHandle);
            obj.VacuumGripper = FrankaVacuumGripper(obj.frankaRobotHandle);
        end
        
        %% Destructor
        function delete(obj)
            if ~isempty(obj.Server)
                obj.Server.stop();
            end
            if ~isempty(obj.frankaRobotHandle)
                franka_robot('delete', obj.frankaRobotHandle);
            end
        end

        %% Franka Robot Automatic Error Recovery
        function automatic_error_recovery(obj, varargin)

            if ~isempty(obj.frankaRobotHandle)
                franka_robot('automatic_error_recovery', obj.frankaRobotHandle, varargin{:});
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end

        end

        function robot_state = robot_state(obj)
            if ~isempty(obj.frankaRobotHandle)
                robot_state = franka_robot('robot_state', obj.frankaRobotHandle);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function joint_poses = joint_poses(obj)
            if ~isempty(obj.frankaRobotHandle)
                joint_poses = franka_robot('joint_poses', obj.frankaRobotHandle);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end
        
        function result = joint_point_to_point_motion(obj, joints_target_configuration, speed_factor)
            if ~isempty(obj.frankaRobotHandle)
                if nargin < 3
                    speed_factor = 0.5; % Default speed factor
                end
                result = franka_robot('joint_point_to_point_motion', obj.frankaRobotHandle, ...
                    joints_target_configuration, speed_factor);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function result = joint_trajectory_motion(obj, positions)
            if ~isempty(obj.frankaRobotHandle)
                
                % Validate dimensions
                [m, ~] = size(positions);
                if m ~= 7
                    error('Positions must be a 7xN array');
                end
                
                result = franka_robot('joint_trajectory_motion', obj.frankaRobotHandle, positions);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function result = setCollisionThresholds(obj, thresholds)
            % Set collision thresholds and apply them to the robot
            obj.Settings.collisionThresholds = thresholds;
            result = franka_robot('set_collision_behavior', obj.frankaRobotHandle, ...
                obj.Settings.collisionThresholds.lower_torque_thresholds_acceleration, ...
                obj.Settings.collisionThresholds.upper_torque_thresholds_acceleration, ...
                obj.Settings.collisionThresholds.lower_torque_thresholds_nominal, ...
                obj.Settings.collisionThresholds.upper_torque_thresholds_nominal, ...
                obj.Settings.collisionThresholds.lower_force_thresholds_acceleration, ...
                obj.Settings.collisionThresholds.upper_force_thresholds_acceleration, ...
                obj.Settings.collisionThresholds.lower_force_thresholds_nominal, ...
                obj.Settings.collisionThresholds.upper_force_thresholds_nominal);
        end

        function thresholds = getCollisionThresholds(obj)
            % Get the current collision thresholds
            thresholds = obj.Settings.collisionThresholds;
        end

        function result = setLoadInertia(obj, loadInertia)
            % Set load inertia parameters and apply them to the robot
            obj.Settings.loadInertia = loadInertia;
            result = franka_robot('set_load', obj.frankaRobotHandle, ...
                obj.Settings.loadInertia.mass, ...
                obj.Settings.loadInertia.center_of_mass, ...
                obj.Settings.loadInertia.load_inertia);
        end

        function inertia = getLoadInertia(obj)
            % Get the current load inertia parameters
            inertia = obj.Settings.loadInertia;
        end

        %% Robot Homing
        function result = robot_homing(obj)
            % Move the robot to its home configuration using point-to-point motion
            % Returns true if the motion was successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                result = obj.joint_point_to_point_motion(obj.DefaultSettings.homeConfiguration,.1);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        %% Reset Settings
        function resetSettings(obj)
            % Reset all settings to their default values
            obj.Settings = obj.DefaultSettings;
            
            % Apply the reset settings to the robot
            obj.applySettings();
        end

        function initialize(obj)
            % Initialize the robot
            franka_robot('initialize_robot', obj.frankaRobotHandle, obj.RobotIP);
        end
    end

    methods (Access = private)
        function applySettings(obj)
            obj.setCollisionThresholds(obj.Settings.collisionThresholds);
            obj.setLoadInertia(obj.Settings.loadInertia);
        end
    end
end