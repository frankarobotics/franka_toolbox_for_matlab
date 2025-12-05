classdef FrankaRobot < handle
    
    properties (Constant, Access = private)
        % Set standard default values for the Remote Server
        DefaultServerUsername = 'franka';
        DefaultServerIP = '172.16.1.2';
        DefaultSSHPort = '22';
        DefaultServerPort = '5001';
    end
    
    properties
        Settings     % FrankaRobotSettings instance (runtime-modifiable settings)
        Server
        Gripper
        VacuumGripper
    end
    
    properties (SetAccess = private)
        RobotIP      % Robot IP address (immutable after construction)
    end
    
    properties (SetAccess = private, Hidden = true)
        frankaRobotHandle
    end
    
    methods
        %% Constructor
        function obj = FrankaRobot(varargin)
            % FrankaRobot Constructor
            %
            % Usage:
            %   robot = FrankaRobot()                          % Local server, default settings
            %   robot = FrankaRobot('Settings', mySettings)    % Local server, custom settings
            %   robot = FrankaRobot('Username', 'franka', 'ServerIP', '172.16.1.2', ...)  % Remote server
            %
            % Parameters:
            %   'Settings'   - FrankaRobotSettings object (optional, uses defaults if not provided)
            %                  Note: robot_ip is captured at construction and cannot be changed.
            %                  Other settings (collision_thresholds, load_inertia, etc.) can be
            %                  modified at runtime via setCollisionThresholds(), setLoadInertia().
            %   'Username'   - SSH username for remote server connection
            %   'ServerIP'   - IP address of remote server host
            %   'SSHPort'    - SSH port (default: '22')
            %   'ServerPort' - RPC server port (default: '5001')
            
            % Create input parser
            p = inputParser;
            
            % Settings can be provided directly - uses FrankaRobotSettings as single source of truth
            addParameter(p, 'Settings', FrankaRobotSettings(), @(x) isa(x, 'FrankaRobotSettings'));
            
            % Server connection parameters
            addParameter(p, 'Username', obj.DefaultServerUsername, @ischar);
            addParameter(p, 'ServerIP', obj.DefaultServerIP, @ischar);
            addParameter(p, 'SSHPort', obj.DefaultSSHPort, @ischar);
            addParameter(p, 'ServerPort', obj.DefaultServerPort, @ischar);
            
            % Parse the inputs
            parse(p, varargin{:});
            params = p.Results;
            
            % Store the Settings object
            obj.Settings = params.Settings;
            
            % Capture robot_ip at construction time (immutable)
            obj.RobotIP = obj.Settings.robot_ip;
            
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
            % Input: thresholds - FrankaRobotCollisionThresholds object
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
            % Get the current collision thresholds
            thresholds = obj.Settings.collision_thresholds;
        end

        function result = setLoadInertia(obj, loadInertia)
            % Set load inertia parameters and apply them to the robot
            % Input: loadInertia - FrankaRobotLoadInertia object
            obj.Settings.load_inertia = loadInertia;
            li = obj.Settings.load_inertia;
            result = franka_robot('set_load', obj.frankaRobotHandle, ...
                li.mass, ...
                li.center_of_mass, ...
                li.inertia_matrix);
        end

        function inertia = getLoadInertia(obj)
            % Get the current load inertia parameters
            inertia = obj.Settings.load_inertia;
        end

        %% Robot Homing
        function result = robot_homing(obj)
            % Move the robot to its home configuration using point-to-point motion
            % Returns true if the motion was successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                result = obj.joint_point_to_point_motion(obj.Settings.home_configuration, 0.1);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        %% Reset Settings
        function resetSettings(obj)
            % Reset all settings to their default values
            obj.Settings = FrankaRobotSettings();
            
            % Apply the reset settings to the robot
            obj.applySettings();
        end

        %% Impedance Control
        function result = setJointImpedance(obj, K_theta)
            % Set the impedance for each joint in the internal controller
            % Input: K_theta - 7-element array of joint stiffness values [Nm/rad]
            %        If not provided, uses Settings.joint_impedance_stiffness
            % Returns: true if successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                if nargin < 2
                    K_theta = obj.Settings.joint_impedance_stiffness;
                else
                    validateattributes(K_theta, {'numeric'}, {'numel', 7, 'positive'});
                    obj.Settings.joint_impedance_stiffness = K_theta(:)';
                end
                result = franka_robot('set_joint_impedance', obj.frankaRobotHandle, K_theta(:)');
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function K_theta = getJointImpedance(obj)
            % Get the current joint impedance stiffness values
            % Returns: 7-element array of joint stiffness values [Nm/rad]
            K_theta = obj.Settings.joint_impedance_stiffness;
        end

        function result = setCartesianImpedance(obj, K_x)
            % Set the Cartesian stiffness/compliance in the internal controller
            % Input: K_x - 6-element array for (x, y, z, roll, pitch, yaw) stiffness
            %              [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]
            %        If not provided, uses Settings.cartesian_impedance_stiffness
            % Returns: true if successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                if nargin < 2
                    K_x = obj.Settings.cartesian_impedance_stiffness;
                else
                    validateattributes(K_x, {'numeric'}, {'numel', 6, 'positive'});
                    obj.Settings.cartesian_impedance_stiffness = K_x(:)';
                end
                result = franka_robot('set_cartesian_impedance', obj.frankaRobotHandle, K_x(:)');
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function K_x = getCartesianImpedance(obj)
            % Get the current Cartesian impedance stiffness values
            % Returns: 6-element array for (x, y, z, roll, pitch, yaw) stiffness
            K_x = obj.Settings.cartesian_impedance_stiffness;
        end

        %% Guiding Mode
        function result = setGuidingMode(obj, guiding_mode, elbow)
            % Lock or unlock guiding mode movement in (x, y, z, roll, pitch, yaw)
            % Input: guiding_mode - 6-element logical array, true = unlocked, false = locked
            %        elbow - logical scalar, true = unlock elbow movement
            % Returns: true if successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                validateattributes(guiding_mode, {'logical'}, {'numel', 6});
                validateattributes(elbow, {'logical'}, {'scalar'});
                result = franka_robot('set_guiding_mode', obj.frankaRobotHandle, guiding_mode(:)', elbow);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        %% Frame Transformations
        function result = setK(obj, EE_T_K)
            % Set the transformation from end effector frame to stiffness frame
            % Input: EE_T_K - 4x4 homogeneous transformation matrix
            %        If not provided, uses Settings.EE_T_K
            % Returns: true if successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                if nargin < 2
                    EE_T_K = obj.Settings.EE_T_K;
                else
                    validateattributes(EE_T_K, {'numeric'}, {'size', [4, 4]});
                    obj.Settings.EE_T_K = EE_T_K;
                end
                % Convert to column-major (MATLAB default) flattened array
                result = franka_robot('set_k', obj.frankaRobotHandle, EE_T_K(:)');
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function EE_T_K = getK(obj)
            % Get the current EE_T_K transformation matrix
            % Returns: 4x4 homogeneous transformation matrix
            EE_T_K = obj.Settings.EE_T_K;
        end

        function result = setEE(obj, NE_T_EE)
            % Set the transformation from nominal end effector to end effector frame
            % Input: NE_T_EE - 4x4 homogeneous transformation matrix
            %        If not provided, uses Settings.NE_T_EE
            % Returns: true if successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                if nargin < 2
                    NE_T_EE = obj.Settings.NE_T_EE;
                else
                    validateattributes(NE_T_EE, {'numeric'}, {'size', [4, 4]});
                    obj.Settings.NE_T_EE = NE_T_EE;
                end
                % Convert to column-major (MATLAB default) flattened array
                result = franka_robot('set_ee', obj.frankaRobotHandle, NE_T_EE(:)');
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function NE_T_EE = getEE(obj)
            % Get the current NE_T_EE transformation matrix
            % Returns: 4x4 homogeneous transformation matrix
            NE_T_EE = obj.Settings.NE_T_EE;
        end

        %% Stop Robot
        function result = stop(obj)
            % Stop all currently running motions
            % Returns: true if successful, false otherwise
            if ~isempty(obj.frankaRobotHandle)
                result = franka_robot('stop_robot', obj.frankaRobotHandle);
            else
                error('Franka Robot server is not connected. Please check the Server Status.');
            end
        end

        function initialize(obj)
            % Initialize the robot using the immutable RobotIP
            franka_robot('initialize_robot', obj.frankaRobotHandle, obj.RobotIP);
        end
    end

    methods (Access = private)
        function applySettings(obj)
            % Apply all settings from the Settings object to the robot
            obj.setCollisionThresholds(obj.Settings.collision_thresholds);
            obj.setLoadInertia(obj.Settings.load_inertia);
            obj.setJointImpedance();      % Uses Settings.joint_impedance_stiffness
            obj.setCartesianImpedance();  % Uses Settings.cartesian_impedance_stiffness
            obj.setEE();                  % Uses Settings.NE_T_EE
            obj.setK();                   % Uses Settings.EE_T_K
        end
    end
end