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
                
                % Construct SSH command based on platform
                if obj.isWindows
                    sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                else
                    sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                end
                
                % Check if executable already exists on remote machine
                [status, ~] = system([sshCmd ' "test -x ' remoteDir '/franka_robot_server"']);
                if status ~= 0
                    obj.setupRemoteWorkspace();
                end
                
                % Run on remote machine
                if obj.isWindows
                    cmd = [sshCmd ' "cd ' remoteDir ' && (./franka_robot_server ' obj.ServerIP ' ' ...
                        obj.ServerPort ' > output.log 2>&1 & echo $! > pidfile)"'];
                else
                    cmd = [sshCmd ' "cd ' remoteDir ' && (./franka_robot_server ' obj.ServerIP ' ' ...
                        obj.ServerPort ' > output.log 2>&1 & echo \$! > pidfile)"'];
                end
                
                % Update paths for remote operation
                obj.logFile = [remoteDir '/output.log'];
                pidFile = [remoteDir '/pidfile'];
            else
                if obj.isWindows
                    error('Local operation is not supported on Windows hosts');
                end
                
                % Check if executable exists
                if ~exist(execPath, 'file')
                    error('Executable not found at: %s', execPath);
                end
                
                % Run on local machine
				% Quote paths to handle spaces in directories/file names
				q = @(p) ['''', p, ''''];
				cmd = [q(execPath) ' ' obj.ServerIP ' ' obj.ServerPort ...
					' > ' q(obj.logFile) ' 2>&1 & echo $! > ' q(pidFile)];
            end

            % Execute command
            [status, ~] = system(cmd);
            if status ~= 0
                error('Failed to start the franka_robot_server');
            end
            
            % Wait and read PID
            pause(1); % Give time for remote operation
            
            if obj.isRemote
                % Wait for remote pidfile to be created (with timeout)
                maxWaitTime = 5; % seconds
                waitTime = 0;
                pidExists = false;
                remoteDir = obj.getRemoteDir();
                
                if obj.isWindows
                    sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                else
                    sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                end
                
                while waitTime < maxWaitTime && ~pidExists
                    [status, ~] = system([sshCmd ' "test -f ' remoteDir '/pidfile"']);
                    pidExists = (status == 0);
                    if ~pidExists
                        pause(0.1);
                        waitTime = waitTime + 0.1;
                    end
                end
                
                if ~pidExists
                    error('Remote PID file was not created within timeout period');
                end
                
                [~, pidStr] = system([sshCmd ' "cat ' remoteDir '/pidfile"']);
                obj.pid = str2double(strtrim(pidStr));
            else
                % Wait for pidfile to be created (with timeout)
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
            
            if obj.pid <= 0
                error('Failed to start franka_robot_server process');
            end
        end
        
        function stop(obj)
            if ~isempty(obj.pid) && obj.isRunning()
                if obj.isRemote
                    if obj.isWindows
                        sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=no -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                    else
                        sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                    end
                    system([sshCmd ' "kill ' num2str(obj.pid) '"']);
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
                if obj.isWindows
                    sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                else
                    sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                end
                [status, ~] = system([sshCmd ' "ps -p ' num2str(obj.pid) '"']);
            else
                [status, ~] = system(['ps -p ', num2str(obj.pid)]);
            end
            running = (status == 0);
        end
        
        function lines = getOutput(obj)
            lines = {};
            if obj.isRemote
                if obj.isWindows
                    sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                else
                    sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                end
                [~, output] = system([sshCmd ' "cat ' obj.logFile '"']);
                if ~isempty(output)
                    lines = strsplit(output, '\n');
                end
            else
                if ~isempty(obj.outputFid) && ~feof(obj.outputFid)
                    while ~feof(obj.outputFid)
                        line = fgetl(obj.outputFid);
                        if ischar(line)
                            lines{end+1} = line;
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
            
            if obj.isWindows
                sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                scpCmd = ['scp.exe -P ' obj.SSHPort];
            else
                sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                scpCmd = ['scp -P ' obj.SSHPort];
            end
            execPath = fullfile(obj.execDir,'franka_robot_server');
            
            % Create remote directory
            system([sshCmd ' "mkdir -p ' remoteDir '"']);
            
            % Copy executable to remote machine
            % Quote local path to handle spaces in directory names
            quotedExecPath = ['"' strrep(execPath,'\','/') '"'];
            [status, ~] = system([scpCmd ' ' quotedExecPath ' ' obj.Username '@' obj.ServerIP ':' remoteDir '/']);
            if status ~= 0
                error('Failed to copy executable to remote machine');
            end
            
            % Copy libfranka folder to remote machine (architecture-specific)
            libfrankaPath = fullfile(franka_toolbox_installation_path_get(), ['libfranka' obj.archSuffix],'build','usr');
            % Quote local path to handle spaces in directory names
            quotedLibfrankaPath = ['"' strrep(libfrankaPath,'\','/') '"'];
            system([sshCmd ' "mkdir -p ' remoteMatlabWs '"']);
            [status, ~] = system([scpCmd ' -r ' quotedLibfrankaPath ' ' obj.Username '@' obj.ServerIP ':' remoteMatlabWs '/']);
            if status ~= 0
                error('Failed to copy libfranka%s folder to remote machine', obj.archSuffix);
            end
        end

        function deleteRemoteWorkspace(obj)
            % Deletes the remote workspace directory and all its contents
            if obj.isRemote
                if obj.isWindows
                    fullRemoteDir = ['/home/' obj.Username '/franka_matlab_ws'];
                    sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                else
                    remoteDir = '~/franka_matlab_ws';
                    sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                end
                
                % Stop the server if it's running
                obj.stop();
                
                % Delete the remote workspace
                if obj.isWindows
                    [status, ~] = system([sshCmd ' "rm -rf ' fullRemoteDir '"']);
                else
                    [status, ~] = system([sshCmd ' "rm -rf ' remoteDir '"']);
                end
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
    end
    
    methods (Access = private)
        function cleanup(obj)
            if obj.isRemote
                remoteDir = obj.getRemoteDir();
                if obj.isWindows
                    sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                    system([sshCmd ' "rm -f ' remoteDir '/output.log ' remoteDir '/pidfile"']);
                else
                    sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
                    system([sshCmd ' "rm -f ' remoteDir '/output.log ' remoteDir '/pidfile"']);
                end
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
        
        function archSuffix = detectRemoteArchitecture(obj)
            % Detect the architecture of the remote machine
            % Returns: '_arm' for aarch64/ARM, '' for x86_64/amd64
            
            if obj.isWindows
                sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
            else
                sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' obj.SSHPort ' ' obj.Username '@' obj.ServerIP];
            end
            
            % Use uname -m to detect architecture
            [status, output] = system([sshCmd ' "uname -m"']);
            
            if status ~= 0
                error('Failed to detect remote architecture. Please check SSH connection.');
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
    end
    
    methods (Static)
        function cleanupRemoteWorkspace(username, serverIP, sshPort)
            % Static method to cleanup the remote workspace
            % Parameters:
            %   username - SSH username for remote connection
            %   serverIP - IP address of remote server
            %   sshPort - SSH port number (as string)
            
            isWindows = ispc();
            if isWindows
                fullRemoteDir = ['/home/' username '/franka_matlab_ws'];
                sshCmd = ['ssh.exe -o ConnectTimeout=5 -o BatchMode=yes -p ' sshPort ' ' username '@' serverIP];
            else
                remoteDir = '~/franka_matlab_ws';
                sshCmd = ['ssh -o ConnectTimeout=5 -o BatchMode=yes -p ' sshPort ' ' username '@' serverIP];
            end

            % Delete the remote workspace
            if isWindows
                [status, ~] = system([sshCmd ' "rm -rf ' fullRemoteDir '"']);
            else
                [status, ~] = system([sshCmd ' "rm -rf ' remoteDir '"']);
            end
            if status ~= 0
                warning('Failed to delete remote workspace directory');
            end
        end
    end
end 
