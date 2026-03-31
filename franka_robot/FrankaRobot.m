classdef FrankaRobot < handle
    %FRANKAROBOT High-level interface to Franka Emika robots
    %
    %   Example (single robot):
    %       robot = FrankaRobot('RobotIP', '172.16.0.2');
    %
    %   Example (multiple robots):
    %       robot1 = FrankaRobot('RobotIP', '172.16.0.2', 'ServerPort', '5001');
    %       robot2 = FrankaRobot('RobotIP', '172.16.0.3', 'ServerPort', '5002');
    
    properties (Constant, Access = private)
        DefaultServerUsername = 'franka';
        DefaultServerIP = '172.16.1.2';
        DefaultSSHPort = '22';
        DefaultServerPort = '5001';
    end
    
    properties
        Settings
        Server
        Gripper
        VacuumGripper
    end
    
    properties (SetAccess = private)
        RobotIP
    end
    
    properties (SetAccess = private, Hidden = true)
        frankaRobotHandle
    end
    
    methods
        function obj = FrankaRobot(varargin)
            % FrankaRobot Constructor
            %
            % Parameters:
            %   'RobotIP'    - IP address of the Franka robot
            %   'Settings'   - FrankaRobotSettings object
            %   'ServerPort' - RPC server port (use different ports for multiple robots)
            %   'ServerIP'   - Remote server IP (triggers SSH mode)
            %   'Username'   - SSH username (default: 'franka')
            %   'SSHPort'    - SSH port (default: '22')
            
            p = inputParser;
            addParameter(p, 'RobotIP', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'Settings', FrankaRobotSettings(), @(x) isa(x, 'FrankaRobotSettings'));
            addParameter(p, 'Username', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'ServerIP', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'SSHPort', obj.DefaultSSHPort, @(x) ischar(x) || isstring(x));
            addParameter(p, 'ServerPort', obj.DefaultServerPort, @(x) ischar(x) || isstring(x));
            parse(p, varargin{:});
            params = p.Results;
            
            obj.Settings = params.Settings;
            obj.RobotIP = char(params.RobotIP);
            if isempty(obj.RobotIP)
                obj.RobotIP = obj.Settings.robot_ip;
            end
            
            % Remote mode if ServerIP is provided
            serverIP = char(params.ServerIP);
            if ~isempty(serverIP)
                username = char(params.Username);
                if isempty(username), username = obj.DefaultServerUsername; end
                obj.Server = FrankaRobotServer(username, serverIP, ...
                    char(params.SSHPort), char(params.ServerPort));
            else
                obj.Server = FrankaRobotServer();
                if ~strcmp(params.ServerPort, obj.DefaultServerPort)
                    obj.Server.setServerPort(char(params.ServerPort));
                end
            end

            try
                obj.Server.start();
            catch ME
                error('FrankaRobot:InitError', 'Failed to start server: %s', ME.message);
            end

            obj.frankaRobotHandle = franka_robot('new', obj.Server.getServerIp(), obj.Server.getServerPort());
            
            try
                obj.initialize();
            catch ME
                obj.Server.stop();
                franka_robot('delete', obj.frankaRobotHandle);
                error('FrankaRobot:InitError', 'Failed to initialize robot: %s', ME.message);
            end

            obj.Gripper = FrankaGripper(obj.frankaRobotHandle);
            obj.VacuumGripper = FrankaVacuumGripper(obj.frankaRobotHandle);
        end
        
        function delete(obj)
            if ~isempty(obj.Server)
                obj.Server.stop();
            end
            if ~isempty(obj.frankaRobotHandle)
                franka_robot('delete', obj.frankaRobotHandle);
            end
        end

        function automatic_error_recovery(obj, varargin)
            obj.checkHandle();
            franka_robot('automatic_error_recovery', obj.frankaRobotHandle, varargin{:});
        end

        function robot_state = robot_state(obj)
            obj.checkHandle();
            robot_state = franka_robot('robot_state', obj.frankaRobotHandle);
        end

        function joint_poses = joint_poses(obj)
            obj.checkHandle();
            joint_poses = franka_robot('joint_poses', obj.frankaRobotHandle);
        end
        
        function result = joint_point_to_point_motion(obj, joints_target_configuration, speed_factor)
            obj.checkHandle();
            if nargin < 3, speed_factor = 0.5; end
            result = franka_robot('joint_point_to_point_motion', obj.frankaRobotHandle, ...
                joints_target_configuration, speed_factor);
        end

        function result = joint_trajectory_motion(obj, positions)
            obj.checkHandle();
            [m, ~] = size(positions);
            if m ~= 7
                error('Positions must be a 7xN array');
            end
            result = franka_robot('joint_trajectory_motion', obj.frankaRobotHandle, positions);
        end

        function result = setCollisionThresholds(obj, thresholds)
            obj.Settings.collision_thresholds = thresholds;
            ct = obj.Settings.collision_thresholds;
            result = franka_robot('set_collision_behavior', obj.frankaRobotHandle, ...
                ct.lower_torque_thresholds_acceleration, ...
                ct.upper_torque_thresholds_acceleration, ...
                ct.lower_torque_thresholds_nominal, ...
                ct.upper_torque_thresholds_nominal, ...
                ct.lower_force_thresholds_acceleration, ...
                ct.upper_force_thresholds_acceleration, ...
                ct.lower_force_thresholds_nominal, ...
                ct.upper_force_thresholds_nominal);
        end

        function thresholds = getCollisionThresholds(obj)
            thresholds = obj.Settings.collision_thresholds;
        end

        function result = setLoadInertia(obj, loadInertia)
            obj.Settings.load_inertia = loadInertia;
            li = obj.Settings.load_inertia;
            result = franka_robot('set_load', obj.frankaRobotHandle, ...
                li.mass, li.center_of_mass, li.inertia_matrix);
        end

        function inertia = getLoadInertia(obj)
            inertia = obj.Settings.load_inertia;
        end

        function result = robot_homing(obj)
            obj.checkHandle();
            result = obj.joint_point_to_point_motion(obj.Settings.home_configuration, 0.1);
        end

        function resetSettings(obj)
            obj.Settings = FrankaRobotSettings();
            obj.applySettings();
        end

        function result = setJointImpedance(obj, K_theta)
            obj.checkHandle();
            if nargin < 2
                K_theta = obj.Settings.joint_impedance_stiffness;
            else
                validateattributes(K_theta, {'numeric'}, {'numel', 7, 'positive'});
                obj.Settings.joint_impedance_stiffness = K_theta(:)';
            end
            result = franka_robot('set_joint_impedance', obj.frankaRobotHandle, K_theta(:)');
        end

        function K_theta = getJointImpedance(obj)
            K_theta = obj.Settings.joint_impedance_stiffness;
        end

        function result = setCartesianImpedance(obj, K_x)
            obj.checkHandle();
            if nargin < 2
                K_x = obj.Settings.cartesian_impedance_stiffness;
            else
                validateattributes(K_x, {'numeric'}, {'numel', 6, 'positive'});
                obj.Settings.cartesian_impedance_stiffness = K_x(:)';
            end
            result = franka_robot('set_cartesian_impedance', obj.frankaRobotHandle, K_x(:)');
        end

        function K_x = getCartesianImpedance(obj)
            K_x = obj.Settings.cartesian_impedance_stiffness;
        end

        function result = setK(obj, EE_T_K)
            obj.checkHandle();
            if nargin < 2
                EE_T_K = obj.Settings.EE_T_K;
            else
                validateattributes(EE_T_K, {'numeric'}, {'size', [4, 4]});
                obj.Settings.EE_T_K = EE_T_K;
            end
            result = franka_robot('set_k', obj.frankaRobotHandle, EE_T_K(:)');
        end

        function EE_T_K = getK(obj)
            EE_T_K = obj.Settings.EE_T_K;
        end

        function result = setEE(obj, NE_T_EE)
            obj.checkHandle();
            if nargin < 2
                NE_T_EE = obj.Settings.NE_T_EE;
            else
                validateattributes(NE_T_EE, {'numeric'}, {'size', [4, 4]});
                obj.Settings.NE_T_EE = NE_T_EE;
            end
            result = franka_robot('set_ee', obj.frankaRobotHandle, NE_T_EE(:)');
        end

        function NE_T_EE = getEE(obj)
            NE_T_EE = obj.Settings.NE_T_EE;
        end

        function result = stop(obj)
            obj.checkHandle();
            result = franka_robot('stop_robot', obj.frankaRobotHandle);
        end
        
        function result = ping(obj)
            % Ping the server to verify connectivity
            obj.checkHandle();
            result = franka_robot('ping', obj.frankaRobotHandle);
        end
        
        function healthy = isHealthy(obj)
            % Check if server connection is healthy via RPC ping
            try
                result = obj.ping();
                healthy = ~isempty(result) && result.port == str2double(obj.Server.getServerPort());
            catch
                healthy = false;
            end
        end

        function reconnect(obj)
            % Reconnect to server after it was restarted
            %
            % Use this method after calling Server.stop() and Server.start()
            % to re-establish the RPC connection.
            
            % Ensure server is running
            if ~obj.Server.isRunning()
                obj.Server.start();
            end
            
            % Delete old handle if exists
            if ~isempty(obj.frankaRobotHandle)
                try
                    franka_robot('delete', obj.frankaRobotHandle);
                catch
                    % Ignore errors from stale handle
                end
            end
            
            % Create new connection
            obj.frankaRobotHandle = franka_robot('new', obj.Server.getServerIp(), obj.Server.getServerPort());
            
            % Reinitialize robot
            obj.initialize();
            
            % Recreate gripper interfaces with new handle
            obj.Gripper = FrankaGripper(obj.frankaRobotHandle);
            obj.VacuumGripper = FrankaVacuumGripper(obj.frankaRobotHandle);
        end

        function initialize(obj)
            franka_robot('initialize_robot', obj.frankaRobotHandle, obj.RobotIP);
        end
    end

    methods (Access = private)
        function checkHandle(obj)
            if isempty(obj.frankaRobotHandle)
                error('FrankaRobot:NotConnected', 'Server not connected');
            end
        end
        
        function applySettings(obj)
            obj.setCollisionThresholds(obj.Settings.collision_thresholds);
            obj.setLoadInertia(obj.Settings.load_inertia);
            obj.setJointImpedance();
            obj.setCartesianImpedance();
            obj.setEE();
            obj.setK();
        end
    end
end
