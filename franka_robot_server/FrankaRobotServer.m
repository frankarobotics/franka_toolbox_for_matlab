classdef FrankaRobotServer < handle
    %FRANKAROBOTSERVER Manages the lifecycle of a Franka robot RPC server
    %   This class handles starting, stopping, and monitoring the franka_robot_server
    %   process. It supports both local and remote (SSH-based) execution with
    %   robust process management including:
    %   - Port-scoped resources (PID files, logs) for multi-instance support
    %   - Orphaned process detection and cleanup
    %   - Process validation to ensure PIDs belong to actual franka_robot_server
    %   - Instance registry to track active servers
    
    properties (Access = private)
        pid
        logFile
        outputFid
        Username    % SSH username for remote connection
        ServerIP = 'localhost'
        SSHPort = '22'
        ServerPort = '5001'
        execDir
        isRemote    % Flag to indicate remote operation
        isWindows   % Flag to indicate Windows host
        archSuffix  % Architecture suffix for bin directory ('', '_arm', etc.)
        startTime   % Timestamp when server was started
    end
    
    properties (Constant, Access = private)
        % Registry of active server instances keyed by "ServerIP:ServerPort"
        % Using persistent variable pattern since MATLAB doesn't support 
        % static mutable properties
        SERVER_PROCESS_NAME = 'franka_robot_server'
    end
    
    methods
        function obj = FrankaRobotServer(Username, ServerIP, SSHPort, ServerPort)
            % Constructor for FrankaRobotServer
            %
            % Usage:
            %   server = FrankaRobotServer()  % Local server on default port 5001
            %   server = FrankaRobotServer(username, serverIP, sshPort, serverPort)  % Remote server
            %
            % For multiple instances, use different ServerPort values.
            
            obj.isRemote = false;
            obj.isWindows = ispc();
            obj.archSuffix = ''; % Default for local execution
            
            if nargin == 4
                obj.isRemote = true;
                
                obj.Username = Username;
                obj.ServerIP = ServerIP;
                obj.SSHPort = SSHPort;
                obj.ServerPort = ServerPort;
                
                % Detect remote architecture
                obj.archSuffix = obj.detectRemoteArchitecture();
            elseif nargin > 0 && nargin < 4
                error('FrankaRobotServer:InvalidArgs', ...
                    'Either provide no arguments (local) or all 4 arguments (remote)');
            end
            
            % Set up execution directory (port-scoped log file set in start())
            obj.execDir = fullfile(franka_toolbox_installation_path_get(), ...
                'franka_robot_server', ['bin', obj.archSuffix]);
        end
        
        function start(obj)
            % Start the server process
            %
            % This method will:
            % 1. Check for orphaned processes on the same port and clean them up
            % 2. Start the franka_robot_server with proper PID tracking
            % 3. Register this instance in the global registry
            
            % Check for and handle any orphaned processes
            obj.cleanupOrphanedProcess();
            
            % Check if already registered and running
            existingServer = FrankaRobotServer.getRegisteredServer(obj.ServerIP, obj.ServerPort);
            if ~isempty(existingServer) && existingServer.isRunning()
                error('FrankaRobotServer:AlreadyRunning', ...
                    'A server is already running on %s:%s', obj.ServerIP, obj.ServerPort);
            end
            
            execPath = fullfile(obj.execDir, 'franka_robot_server');
            pidFile = obj.getPidFilePath();
            obj.logFile = obj.getLogFilePath();
            
            if obj.isRemote
                obj.startRemote(execPath, pidFile);
            else
                obj.startLocal(execPath, pidFile);
            end
            
            if isempty(obj.pid) || obj.pid <= 0
                error('FrankaRobotServer:StartFailed', ...
                    'Failed to start franka_robot_server process');
            end
            
            obj.startTime = datetime('now');
            
            % Register this server instance
            FrankaRobotServer.registerServer(obj);
        end
        
        function stop(obj)
            % Stop the server process gracefully
            if ~isempty(obj.pid) && obj.isRunning()
                if obj.isRemote
                    % Try graceful SIGTERM first, then SIGKILL
                    obj.sshExec(['kill -TERM ' num2str(obj.pid) ' 2>/dev/null || kill -KILL ' num2str(obj.pid) ' 2>/dev/null']);
                else
                    system(['kill -TERM ', num2str(obj.pid), ' 2>/dev/null || kill -KILL ', num2str(obj.pid), ' 2>/dev/null']);
                end
                
                % Wait briefly for process to terminate
                for i = 1:10
                    if ~obj.isRunning()
                        break;
                    end
                    pause(0.1);
                end
            end
            
            % Unregister and cleanup
            FrankaRobotServer.unregisterServer(obj);
            obj.cleanup();
        end
        
        function running = isRunning(obj)
            % Check if the server process is running
            % Uses both PID check and process name validation
            if isempty(obj.pid)
                running = false;
                return;
            end
            
            if obj.isRemote
                running = obj.isProcessRunningRemote();
            else
                running = obj.isProcessRunningLocal();
            end
        end
        
        function healthy = isHealthy(obj, clientHandle)
            % Check if server is healthy by attempting an RPC ping
            %
            % Usage:
            %   healthy = server.isHealthy(frankaRobotHandle)
            %
            % This performs an actual RPC call to verify the server is responsive,
            % not just that the process exists.
            
            if nargin < 2
                % If no client handle provided, fall back to isRunning()
                healthy = obj.isRunning();
                return;
            end
            
            try
                result = franka_robot('ping', clientHandle);
                healthy = ~isempty(result) && result.port == str2double(obj.ServerPort);
            catch
                healthy = false;
            end
        end
        
        function lines = getOutput(obj)
            % Get server log output
            lines = {};
            if obj.isRemote
                [~, output] = obj.sshExec(['cat ' obj.logFile ' 2>/dev/null']);
                if ~isempty(output)
                    lines = strsplit(output, '\n');
                end
            else
                if ~isempty(obj.outputFid) && ~feof(obj.outputFid)
                    while ~feof(obj.outputFid)
                        line = fgetl(obj.outputFid);
                        if ischar(line)
                            lines{end+1} = line; %#ok<AGROW>
                        end
                    end
                end
            end
        end
        
        function delete(obj)
            % Destructor - ensure cleanup on object deletion
            obj.stop();
        end
        
        function setupRemoteWorkspace(obj)
            % Sets up the remote workspace by creating directories and copying necessary files
            
            remoteDir = obj.getRemoteDir();
            remoteMatlabWs = obj.getRemoteMatlabWsDir();
            scpOpts = struct('recursive', false, 'nothrow', false);
            scpOptsRecursive = struct('recursive', true, 'nothrow', false);
            
            % Create remote directories
            obj.sshExec(['mkdir -p ' remoteDir]);
            obj.sshExec(['mkdir -p ' remoteMatlabWs]);
            
            % Copy executable to remote machine
            execPath = fullfile(obj.execDir, 'franka_robot_server');
            franka_toolbox_scp(execPath, [':' remoteDir '/'], ...
                obj.Username, obj.ServerIP, obj.SSHPort, scpOpts);
            
            % Copy libfranka folder to remote machine (architecture-specific)
            libfrankaPath = fullfile(franka_toolbox_installation_path_get(), ...
                ['libfranka' obj.archSuffix], 'build', 'usr');
            franka_toolbox_scp(libfrankaPath, [':' remoteMatlabWs '/'], ...
                obj.Username, obj.ServerIP, obj.SSHPort, scpOptsRecursive);
        end

        function deleteRemoteWorkspace(obj)
            % Deletes the remote workspace directory and all its contents
            if obj.isRemote
                % Stop the server if it's running
                obj.stop();
                
                % Delete the remote workspace
                remoteWsRoot = obj.getRemoteWorkspaceRoot();
                [status, ~] = obj.sshExec(['rm -rf ' remoteWsRoot]);
                if status ~= 0
                    warning('FrankaRobotServer:CleanupFailed', ...
                        'Failed to delete remote workspace directory');
                end
            else
                warning('FrankaRobotServer:NotRemote', ...
                    'deleteRemoteWorkspace only applies to remote configurations');
            end
        end

        % Getter methods
        function value = getUsername(obj)
            value = obj.Username;
        end

        function value = getServerIp(obj)
            value = obj.ServerIP;
        end

        function value = getSshPort(obj)
            value = obj.SSHPort;
        end

        function value = getServerPort(obj)
            value = obj.ServerPort;
        end

        function value = getPid(obj)
            value = obj.pid;
        end

        % Setter methods (only before start)
        function setUsername(obj, value)
            if obj.isRunning()
                error('FrankaRobotServer:AlreadyRunning', 'Cannot modify while server is running');
            end
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.Username = char(value);
        end

        function setServerIp(obj, value)
            if obj.isRunning()
                error('FrankaRobotServer:AlreadyRunning', 'Cannot modify while server is running');
            end
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.ServerIP = char(value);
        end

        function setSshPort(obj, value)
            if obj.isRunning()
                error('FrankaRobotServer:AlreadyRunning', 'Cannot modify while server is running');
            end
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.SSHPort = char(value);
        end

        function setServerPort(obj, value)
            if obj.isRunning()
                error('FrankaRobotServer:AlreadyRunning', 'Cannot modify while server is running');
            end
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.ServerPort = char(value);
        end

        function cleanupRemote(obj)
            % Public method to cleanup the remote workspace using instance properties
            if obj.isRunning()
                warning('FrankaRobotServer:ServerRunning', ...
                    'Cannot cleanup remote workspace while server is running');
                return;
            end
            
            if obj.isRemote
                FrankaRobotServer.cleanupRemoteWorkspace(obj.Username, obj.ServerIP, obj.SSHPort);
            else
                warning('FrankaRobotServer:NotRemote', ...
                    'cleanupRemote only applies to remote configurations');
            end
        end
        
        function s = status(obj)
            % Returns a struct with diagnostic information about the server
            %
            % Returns:
            %   s.running     - true if server process is running
            %   s.pid         - process ID (empty if not started)
            %   s.mode        - 'local' or 'remote'
            %   s.serverIP    - server IP address
            %   s.port        - server port
            %   s.logFile     - path to log file
            %   s.startTime   - when the server was started
            %   s.validated   - true if PID confirmed as franka_robot_server
            
            s = struct();
            s.running = obj.isRunning();
            s.pid = obj.pid;
            s.mode = ternary(obj.isRemote, 'remote', 'local');
            s.serverIP = obj.ServerIP;
            s.port = obj.ServerPort;
            s.logFile = obj.logFile;
            s.startTime = obj.startTime;
            
            if ~isempty(obj.pid)
                s.validated = obj.isOurProcess(obj.pid);
            else
                s.validated = false;
            end
            
            if obj.isRemote
                s.remoteUser = obj.Username;
                s.sshPort = obj.SSHPort;
                s.architecture = obj.archSuffix;
            end
            
            function result = ternary(cond, trueVal, falseVal)
                if cond
                    result = trueVal;
                else
                    result = falseVal;
                end
            end
        end
    end
    
    methods (Access = private)
        %% Port-scoped file paths
        function path = getPidFilePath(obj)
            % Get port-scoped PID file path
            if obj.isRemote
                remoteDir = obj.getRemoteDir();
                path = sprintf('%s/franka_server_%s.pid', remoteDir, obj.ServerPort);
            else
                path = fullfile(obj.execDir, sprintf('franka_server_%s.pid', obj.ServerPort));
            end
        end
        
        function path = getLogFilePath(obj)
            % Get port-scoped log file path
            if obj.isRemote
                remoteDir = obj.getRemoteDir();
                path = sprintf('%s/franka_server_%s.log', remoteDir, obj.ServerPort);
            else
                path = fullfile(obj.execDir, sprintf('franka_server_%s.log', obj.ServerPort));
            end
        end
        
        %% Start helpers
        function startLocal(obj, execPath, pidFile)
            if obj.isWindows
                error('FrankaRobotServer:Unsupported', ...
                    'Local operation is not supported on Windows hosts');
            end
            
            % Check if executable exists
            if ~exist(execPath, 'file')
                error('FrankaRobotServer:NotFound', ...
                    ['Executable not found at: %s\n' ...
                     'Run franka_robot_server_build() to build the server.'], execPath);
            end
            
            % Run on local machine with proper quoting
            q = @(p) ['''', p, ''''];
            cmd = [q(execPath) ' ' obj.ServerIP ' ' obj.ServerPort ...
                ' > ' q(obj.logFile) ' 2>&1 & echo $! > ' q(pidFile)];
            
            [status, ~] = franka_toolbox_local_exec(cmd, obj.execDir);
            if status ~= 0
                error('FrankaRobotServer:StartFailed', 'Failed to start the franka_robot_server');
            end
            
            % Wait and read PID with validation
            pause(0.5);
            obj.readLocalPid(pidFile);
        end
        
        function startRemote(obj, ~, pidFile)
            % Set up remote workspace if not already done
            remoteDir = obj.getRemoteDir();
            
            % Check if executable already exists on remote machine
            [status, ~] = obj.sshExec(['test -x ' remoteDir '/franka_robot_server']);
            if status ~= 0
                obj.setupRemoteWorkspace();
            end
            
            % Run on remote machine
            if obj.isWindows
                remoteCmd = ['cd ' remoteDir ' && (./franka_robot_server ' obj.ServerIP ' ' ...
                    obj.ServerPort ' > ' obj.logFile ' 2>&1 & echo $! > ' pidFile ')'];
            else
                remoteCmd = ['cd ' remoteDir ' && (./franka_robot_server ' obj.ServerIP ' ' ...
                    obj.ServerPort ' > ' obj.logFile ' 2>&1 & echo \$! > ' pidFile ')'];
            end
            [status, ~] = obj.sshExec(remoteCmd);
            
            if status ~= 0
                error('FrankaRobotServer:StartFailed', 'Failed to start the franka_robot_server');
            end
            
            % Wait and read PID
            pause(0.5);
            obj.readRemotePid(pidFile);
        end
        
        %% Process validation
        function isOurs = isOurProcess(obj, pid)
            % Verify that the given PID belongs to a franka_robot_server process
            if obj.isRemote
                % Read /proc/PID/cmdline on remote
                [status, cmdline] = obj.sshExec(sprintf('cat /proc/%d/cmdline 2>/dev/null | tr "\\0" " "', pid));
                isOurs = status == 0 && contains(cmdline, obj.SERVER_PROCESS_NAME);
            else
                % Read /proc/PID/cmdline locally
                cmdlineFile = sprintf('/proc/%d/cmdline', pid);
                if exist(cmdlineFile, 'file')
                    cmdline = fileread(cmdlineFile);
                    isOurs = contains(cmdline, obj.SERVER_PROCESS_NAME);
                else
                    isOurs = false;
                end
            end
        end
        
        function running = isProcessRunningLocal(obj)
            % Check if local process is running and is our server
            [status, ~] = system(['ps -p ', num2str(obj.pid), ' > /dev/null 2>&1']);
            if status ~= 0
                running = false;
                return;
            end
            % Validate it's actually our process
            running = obj.isOurProcess(obj.pid);
        end
        
        function running = isProcessRunningRemote(obj)
            % Check if remote process is running and is our server
            [status, ~] = obj.sshExec(['ps -p ' num2str(obj.pid) ' > /dev/null 2>&1']);
            if status ~= 0
                running = false;
                return;
            end
            % Validate it's actually our process
            running = obj.isOurProcess(obj.pid);
        end
        
        %% Orphan cleanup
        function cleanupOrphanedProcess(obj)
            % Check for and cleanup any orphaned server process on our port
            
            % First check if there's a stale PID file
            pidFile = obj.getPidFilePath();
            pid = obj.readPidFromFile(pidFile);
            
            if ~isempty(pid) && pid > 0
                if obj.isOurProcess(pid)
                    % Found orphaned process, kill it
                    warning('FrankaRobotServer:OrphanedProcess', ...
                        'Found orphaned franka_robot_server (PID %d) on port %s, cleaning up...', ...
                        pid, obj.ServerPort);
                    
                    if obj.isRemote
                        obj.sshExec(sprintf('kill -KILL %d 2>/dev/null', pid));
                    else
                        system(sprintf('kill -KILL %d 2>/dev/null', pid));
                    end
                    pause(0.3);
                end
            end
            
            % Also check if anything is listening on our port (belt and suspenders)
            obj.checkPortAvailability();
        end
        
        function checkPortAvailability(obj)
            % Verify the port is not in use by another process
            if obj.isRemote
                % Use ss or netstat to check port
                [status, output] = obj.sshExec(sprintf(...
                    'ss -tlnp 2>/dev/null | grep ":%s " || netstat -tlnp 2>/dev/null | grep ":%s "', ...
                    obj.ServerPort, obj.ServerPort));
                portInUse = (status == 0 && ~isempty(strtrim(output)));
            else
                [status, output] = system(sprintf(...
                    'ss -tlnp 2>/dev/null | grep ":%s " || netstat -tlnp 2>/dev/null | grep ":%s "', ...
                    obj.ServerPort, obj.ServerPort));
                portInUse = (status == 0 && ~isempty(strtrim(output)));
            end
            
            if portInUse
                error('FrankaRobotServer:PortInUse', ...
                    'Port %s is already in use. Check for running processes or choose a different port.', ...
                    obj.ServerPort);
            end
        end
        
        function pid = readPidFromFile(obj, pidFile)
            % Read PID from file (local or remote)
            pid = [];
            if obj.isRemote
                [status, pidStr] = obj.sshExec(['cat ' pidFile ' 2>/dev/null']);
                if status == 0 && ~isempty(pidStr)
                    pid = str2double(strtrim(pidStr));
                end
            else
                if exist(pidFile, 'file')
                    pidStr = fileread(pidFile);
                    pid = str2double(strtrim(pidStr));
                end
            end
            
            if isnan(pid)
                pid = [];
            end
        end
        
        %% Cleanup
        function cleanup(obj)
            % Clean up resources (PID file, log file handle)
            pidFile = obj.getPidFilePath();
            
            if obj.isRemote
                remoteDir = obj.getRemoteDir();
                obj.sshExec(['rm -f ' pidFile ' ' obj.logFile]);
            else
                if ~isempty(obj.outputFid)
                    fclose(obj.outputFid);
                    obj.outputFid = [];
                end
                if exist(pidFile, 'file')
                    delete(pidFile);
                end
                if exist(obj.logFile, 'file')
                    delete(obj.logFile);
                end
            end
            obj.pid = [];
        end
        
        function [status, output] = sshExec(obj, cmd, options)
            % Execute SSH command using centralized utility
            if nargin < 3
                options = struct('nothrow', true);
            end
            [status, output] = franka_toolbox_ssh_exec(cmd, obj.Username, ...
                obj.ServerIP, obj.SSHPort, options);
        end
        
        function validateSSHConnection(obj)
            % Validate SSH connection before attempting remote operations
            opts = struct('nothrow', false, 'timeout', 5);
            try
                franka_toolbox_ssh_exec('echo ok', obj.Username, obj.ServerIP, obj.SSHPort, opts);
            catch
                error('FrankaRobotServer:SSHFailed', ...
                    ['SSH connection to %s@%s:%s failed.\n' ...
                     'Ensure:\n' ...
                     '  1. The remote host is reachable\n' ...
                     '  2. SSH key authentication is configured (BatchMode is used)\n' ...
                     '  3. Username and port are correct'], ...
                     obj.Username, obj.ServerIP, obj.SSHPort);
            end
        end
        
        function archSuffix = detectRemoteArchitecture(obj)
            % Detect the architecture of the remote machine
            
            % Validate connection first
            obj.validateSSHConnection();
            
            % Use uname -m to detect architecture
            [status, output] = obj.sshExec('uname -m');
            
            if status ~= 0
                error('FrankaRobotServer:ArchDetectFailed', 'Failed to detect remote architecture.');
            end
            
            % Parse architecture string
            arch = strtrim(output);
            
            % Map architecture to suffix
            switch arch
                case {'aarch64', 'arm64', 'armv8l'}
                    archSuffix = '_arm';
                case {'x86_64', 'amd64', 'i686', 'i386'}
                    archSuffix = '';
                otherwise
                    warning('FrankaRobotServer:UnknownArch', ...
                        'Unknown architecture "%s", assuming x86_64', arch);
                    archSuffix = '';
            end
        end
        
        function remoteDir = getRemoteDir(obj)
            % Get the remote directory path based on platform and architecture
            if obj.isWindows
                remoteDir = ['/home/' obj.Username '/franka_matlab_ws/franka_robot_server/bin' obj.archSuffix];
            else
                remoteDir = ['~/franka_matlab_ws/franka_robot_server/bin' obj.archSuffix];
            end
        end
        
        function remoteMatlabWs = getRemoteMatlabWsDir(obj)
            % Get the remote MATLAB workspace directory path
            if obj.isWindows
                remoteMatlabWs = ['/home/' obj.Username '/franka_matlab_ws/libfranka' obj.archSuffix '/build'];
            else
                remoteMatlabWs = ['~/franka_matlab_ws/libfranka' obj.archSuffix '/build'];
            end
        end
        
        function remoteWsRoot = getRemoteWorkspaceRoot(obj)
            % Get the remote workspace root directory
            if obj.isWindows
                remoteWsRoot = ['/home/' obj.Username '/franka_matlab_ws'];
            else
                remoteWsRoot = '~/franka_matlab_ws';
            end
        end
        
        function readRemotePid(obj, pidFile)
            % Wait for remote pidfile and read PID with validation
            maxWaitTime = 5; % seconds
            waitTime = 0;
            pidExists = false;
            
            while waitTime < maxWaitTime && ~pidExists
                [status, ~] = obj.sshExec(['test -f ' pidFile]);
                pidExists = (status == 0);
                if ~pidExists
                    pause(0.1);
                    waitTime = waitTime + 0.1;
                end
            end
            
            if ~pidExists
                error('FrankaRobotServer:PidFileTimeout', ...
                    'PID file was not created within timeout period');
            end
            
            [~, pidStr] = obj.sshExec(['cat ' pidFile]);
            obj.pid = str2double(strtrim(pidStr));
            
            % Validate the process is actually ours
            pause(0.2); % Give process time to initialize
            if ~obj.isOurProcess(obj.pid)
                obj.pid = [];
                error('FrankaRobotServer:ProcessValidationFailed', ...
                    'Started process is not a franka_robot_server');
            end
        end
        
        function readLocalPid(obj, pidFile)
            % Wait for local pidfile and read PID with validation
            maxWaitTime = 5; % seconds
            waitTime = 0;
            
            while ~exist(pidFile, 'file') && waitTime < maxWaitTime
                pause(0.1);
                waitTime = waitTime + 0.1;
            end
            
            if ~exist(pidFile, 'file')
                error('FrankaRobotServer:PidFileTimeout', ...
                    'PID file was not created within timeout period');
            end
            
            obj.pid = str2double(fileread(pidFile));
            obj.outputFid = fopen(obj.logFile, 'r');
            
            % Validate the process is actually ours
            pause(0.2); % Give process time to initialize
            if ~obj.isOurProcess(obj.pid)
                obj.pid = [];
                error('FrankaRobotServer:ProcessValidationFailed', ...
                    'Started process is not a franka_robot_server');
            end
        end
    end
    
    methods (Static)
        function cleanupRemoteWorkspace(username, serverIP, sshPort)
            % Static method to cleanup the remote workspace
            %
            % Parameters:
            %   username - SSH username for remote connection
            %   serverIP - IP address of remote server
            %   sshPort - SSH port number (as string)
            
            if ispc()
                remoteWsRoot = ['/home/' username '/franka_matlab_ws'];
            else
                remoteWsRoot = '~/franka_matlab_ws';
            end
            
            % Use centralized SSH utility
            opts = struct('nothrow', true);
            [status, ~] = franka_toolbox_ssh_exec(['rm -rf ' remoteWsRoot], ...
                username, serverIP, sshPort, opts);
            if status ~= 0
                warning('FrankaRobotServer:CleanupFailed', ...
                    'Failed to delete remote workspace directory');
            end
        end
        
        function servers = listActiveServers()
            % List all active server instances
            %
            % Returns a struct array with info about each registered server
            
            registry = FrankaRobotServer.getRegistry();
            keys = registry.keys();
            servers = struct('key', {}, 'serverIP', {}, 'port', {}, 'pid', {}, 'running', {});
            
            for i = 1:length(keys)
                key = keys{i};
                server = registry(key);
                if isvalid(server)
                    servers(end+1).key = key; %#ok<AGROW>
                    servers(end).serverIP = server.ServerIP;
                    servers(end).port = server.ServerPort;
                    servers(end).pid = server.pid;
                    servers(end).running = server.isRunning();
                end
            end
        end
        
        function stopAllServers()
            % Stop all registered server instances
            %
            % Useful for cleanup, e.g., in a finish.m script
            
            registry = FrankaRobotServer.getRegistry();
            keys = registry.keys();
            
            for i = 1:length(keys)
                server = registry(keys{i});
                if isvalid(server)
                    try
                        server.stop();
                    catch ME
                        warning('FrankaRobotServer:StopFailed', ...
                            'Failed to stop server %s: %s', keys{i}, ME.message);
                    end
                end
            end
        end
    end
    
    methods (Static, Access = private)
        function registry = getRegistry()
            % Get the persistent registry of server instances
            persistent serverRegistry
            if isempty(serverRegistry)
                serverRegistry = containers.Map('KeyType', 'char', 'ValueType', 'any');
            end
            registry = serverRegistry;
        end
        
        function registerServer(server)
            % Register a server instance in the global registry
            key = sprintf('%s:%s', server.ServerIP, server.ServerPort);
            registry = FrankaRobotServer.getRegistry();
            registry(key) = server; %#ok<NASGU>
        end
        
        function unregisterServer(server)
            % Remove a server instance from the registry
            key = sprintf('%s:%s', server.ServerIP, server.ServerPort);
            registry = FrankaRobotServer.getRegistry();
            if registry.isKey(key)
                registry.remove(key);
            end
        end
        
        function server = getRegisteredServer(serverIP, serverPort)
            % Get a registered server by IP and port
            key = sprintf('%s:%s', serverIP, serverPort);
            registry = FrankaRobotServer.getRegistry();
            if registry.isKey(key)
                server = registry(key);
                if ~isvalid(server)
                    registry.remove(key);
                    server = [];
                end
            else
                server = [];
            end
        end
    end
end
