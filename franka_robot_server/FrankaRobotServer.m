classdef FrankaRobotServer < handle
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
    end
    
    methods
        function obj = FrankaRobotServer(Username, ServerIP, SSHPort, ServerPort)
            % Constructor for FrankaRobotServer
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
            end
            
            % Set up execution directory and log file
            obj.execDir = fullfile(franka_toolbox_installation_path_get(), 'franka_robot_server', ['bin', obj.archSuffix]);
            obj.logFile = fullfile(obj.execDir, 'output.log');
        end
        
        function start(obj)
            % Start the executable and save its PID
            execPath = fullfile(obj.execDir,'franka_robot_server');
            pidFile = fullfile(obj.execDir, 'pidfile');
            
            if obj.isRemote
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
                        obj.ServerPort ' > output.log 2>&1 & echo $! > pidfile)'];
                else
                    remoteCmd = ['cd ' remoteDir ' && (./franka_robot_server ' obj.ServerIP ' ' ...
                        obj.ServerPort ' > output.log 2>&1 & echo \$! > pidfile)'];
                end
                [status, ~] = obj.sshExec(remoteCmd);
                
                if status ~= 0
                    error('Failed to start the franka_robot_server');
                end
                
                % Update log path for remote operation
                obj.logFile = [remoteDir '/output.log'];
                
                % Wait and read PID
                pause(1); % Give time for process to start
                obj.readRemotePid();
            else
                if obj.isWindows
                    error('Local operation is not supported on Windows hosts');
                end
                
                % Check if executable exists
                if ~exist(execPath, 'file')
                    error(['Executable not found at: %s\n' ...
                           'Run franka_robot_server_build() to build the server.'], execPath);
                end
                
                % Run on local machine
                q = @(p) ['''', p, ''''];
                cmd = [q(execPath) ' ' obj.ServerIP ' ' obj.ServerPort ...
                    ' > ' q(obj.logFile) ' 2>&1 & echo $! > ' q(pidFile)];
                
                [status, ~] = franka_toolbox_local_exec(cmd, obj.execDir);
                if status ~= 0
                    error('Failed to start the franka_robot_server');
                end
                
                % Wait and read PID
                pause(1);
                obj.readLocalPid(pidFile);
            end
            
            if obj.pid <= 0
                error('Failed to start franka_robot_server process');
            end
        end
        
        function stop(obj)
            if ~isempty(obj.pid) && obj.isRunning()
                if obj.isRemote
                    obj.sshExec(['kill ' num2str(obj.pid)]);
                else
                    system(['kill ', num2str(obj.pid)]);
                end
            end
            obj.cleanup();
        end
        
        function running = isRunning(obj)
            if isempty(obj.pid)
                running = false;
                return;
            end
            if obj.isRemote
                [status, ~] = obj.sshExec(['ps -p ' num2str(obj.pid)]);
            else
                [status, ~] = system(['ps -p ', num2str(obj.pid)]);
            end
            running = (status == 0);
        end
        
        function lines = getOutput(obj)
            lines = {};
            if obj.isRemote
                [~, output] = obj.sshExec(['cat ' obj.logFile]);
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
                    warning('Failed to delete remote workspace directory');
                end
            else
                warning('deleteRemoteWorkspace only applies to remote configurations');
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

        % Setter methods
        function setUsername(obj, value)
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.Username = value;
        end

        function setServerIp(obj, value)
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.ServerIP = value;
        end

        function setSshPort(obj, value)
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.SSHPort = value;
        end

        function setServerPort(obj, value)
            validateattributes(value, {'char', 'string'}, {'nonempty'});
            obj.ServerPort = value;
        end

        function cleanupRemote(obj)
            % Public method to cleanup the remote workspace using instance properties
            if obj.isRunning()
                warning('Cannot cleanup remote workspace while server is running');
                return;
            end
            
            if obj.isRemote
                FrankaRobotServer.cleanupRemoteWorkspace(obj.Username, obj.ServerIP, obj.SSHPort);
            else
                warning('cleanupRemote only applies to remote configurations');
            end
        end
        
        function s = status(obj)
            % Returns a struct with diagnostic information about the server
            %   s = server.status()
            %
            % Returns:
            %   s.running   - true if server process is running
            %   s.pid       - process ID (empty if not started)
            %   s.mode      - 'local' or 'remote'
            %   s.serverIP  - server IP address
            %   s.port      - server port
            %   s.logFile   - path to log file
            
            s = struct();
            s.running = obj.isRunning();
            s.pid = obj.pid;
            s.mode = conditional(obj.isRemote, 'remote', 'local');
            s.serverIP = obj.ServerIP;
            s.port = obj.ServerPort;
            s.logFile = obj.logFile;
            
            if obj.isRemote
                s.remoteUser = obj.Username;
                s.sshPort = obj.SSHPort;
                s.architecture = obj.archSuffix;
            end
            
            function result = conditional(cond, trueVal, falseVal)
                if cond
                    result = trueVal;
                else
                    result = falseVal;
                end
            end
        end
    end
    
    methods (Access = private)
        function cleanup(obj)
            if obj.isRemote
                remoteDir = obj.getRemoteDir();
                obj.sshExec(['rm -f ' remoteDir '/output.log ' remoteDir '/pidfile']);
            else
                if ~isempty(obj.outputFid)
                    fclose(obj.outputFid);
                    obj.outputFid = [];
                end
                if exist(obj.logFile, 'file')
                    delete(obj.logFile);
                end
                if exist(fullfile(obj.execDir, 'pidfile'), 'file')
                    delete(fullfile(obj.execDir, 'pidfile'));
                end
            end
            obj.pid = [];
        end
        
        function [status, output] = sshExec(obj, cmd, options)
            % Execute SSH command using centralized utility
            % Wrapper around franka_toolbox_ssh_exec with instance properties
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
                error(['SSH connection to %s@%s:%s failed.\n' ...
                       'Ensure:\n' ...
                       '  1. The remote host is reachable\n' ...
                       '  2. SSH key authentication is configured (BatchMode is used)\n' ...
                       '  3. Username and port are correct'], ...
                       obj.Username, obj.ServerIP, obj.SSHPort);
            end
        end
        
        function archSuffix = detectRemoteArchitecture(obj)
            % Detect the architecture of the remote machine
            % Returns: '_arm' for aarch64/ARM, '' for x86_64/amd64
            
            % Validate connection first
            obj.validateSSHConnection();
            
            % Use uname -m to detect architecture
            [status, output] = obj.sshExec('uname -m');
            
            if status ~= 0
                error('Failed to detect remote architecture.');
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
                    warning('Unknown architecture "%s", assuming x86_64', arch);
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
            % Get the remote MATLAB workspace directory path based on platform and architecture
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
        
        function readRemotePid(obj)
            % Wait for remote pidfile to be created and read PID
            maxWaitTime = 5; % seconds
            waitTime = 0;
            pidExists = false;
            remoteDir = obj.getRemoteDir();
            
            while waitTime < maxWaitTime && ~pidExists
                [status, ~] = obj.sshExec(['test -f ' remoteDir '/pidfile']);
                pidExists = (status == 0);
                if ~pidExists
                    pause(0.1);
                    waitTime = waitTime + 0.1;
                end
            end
            
            if ~pidExists
                error('Remote PID file was not created within timeout period');
            end
            
            [~, pidStr] = obj.sshExec(['cat ' remoteDir '/pidfile']);
            obj.pid = str2double(strtrim(pidStr));
        end
        
        function readLocalPid(obj, pidFile)
            % Wait for local pidfile to be created and read PID
            maxWaitTime = 5; % seconds
            waitTime = 0;
            
            while ~exist(pidFile, 'file') && waitTime < maxWaitTime
                pause(0.1);
                waitTime = waitTime + 0.1;
            end
            
            if ~exist(pidFile, 'file')
                error('PID file was not created within timeout period');
            end
            
            obj.pid = str2double(fileread(pidFile));
            obj.outputFid = fopen(obj.logFile, 'r');
        end
    end
    
    methods (Static)
        function cleanupRemoteWorkspace(username, serverIP, sshPort)
            % Static method to cleanup the remote workspace
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
                warning('Failed to delete remote workspace directory');
            end
        end
    end
end 
