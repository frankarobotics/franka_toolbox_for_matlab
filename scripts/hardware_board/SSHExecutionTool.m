classdef SSHExecutionTool < target.ExecutionTool
    %SSHEXECUTIONTOOL Execute applications on remote Linux target via SSH
    %   This class provides the MATLAB service interface for starting,
    %   stopping, and monitoring applications on a remote Linux target
    %   using SSH (key-based authentication only).
    %
    %   Required properties to be set before use:
    %     - SSHUser: SSH username on the target
    %     - SSHHost: IP address or hostname of the target
    %     - RemoteDir: Directory on target where executable is deployed
    %
    %   Example:
    %     tool = SSHExecutionTool();
    %     tool.SSHUser = 'franka';
    %     tool.SSHHost = '192.168.1.100';
    %     tool.RemoteDir = '/home/franka/simwork';
    
    properties (Access = public)
        SSHUser (1,1) string = "servobox-usr"
        SSHHost (1,1) string = "192.168.122.100"
        RemoteDir (1,1) string = "/tmp/simwork"
        SSHOptions (1,1) string = "-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
    end
    
    properties (Access = private)
        RemotePID (1,1) double = -1
        IsRunning (1,1) logical = false
    end
    
    methods
        function errFlag = startApplication(this)
            %STARTAPPLICATION Start the application on the remote target
            %   Copies the executable and required libraries to the remote target
            %   and runs the application with proper library path.
            %   Returns errFlag = false on success, true on failure.
            
            errFlag = false;
            
            % Get the application path from the base class
            appPath = this.Application;
            if isempty(appPath)
                disp('SSHExecutionTool: No application specified.');
                errFlag = true;
                return;
            end
            
            [~, exeBaseName, exeExt] = fileparts(appPath);
            fullExeName = char(strcat(exeBaseName, exeExt));
            
            % Convert all string properties to char for reliable sprintf
            sshOpts = char(this.SSHOptions);
            sshUser = char(this.SSHUser);
            sshHost = char(this.SSHHost);
            remoteDir = char(this.RemoteDir);
            remoteLibDir = [remoteDir, '/lib'];
            
            % 1) Setup remote directories
            mkdirCmd = sprintf('ssh %s %s@%s ''mkdir -p %s %s''', ...
                sshOpts, sshUser, sshHost, remoteDir, remoteLibDir);
            
            fprintf('SSHExecutionTool: Creating remote directories on %s...\n', sshHost);
            [status, result] = system(mkdirCmd);
            if status ~= 0
                fprintf('SSHExecutionTool: Failed to create remote directory.\n%s\n', result);
                errFlag = true;
                return;
            end
            
            % 2) Copy executable
            fprintf('SSHExecutionTool: Deploying %s to %s@%s:%s...\n', fullExeName, sshUser, sshHost, remoteDir);
            scpExeCmd = sprintf('scp %s "%s" %s@%s:%s/', ...
                sshOpts, char(appPath), sshUser, sshHost, remoteDir);
            [status, result] = system(scpExeCmd);
            if status ~= 0
                fprintf('SSHExecutionTool: Failed to copy application to target.\n%s\n', result);
                errFlag = true;
                return;
            end
            
            % 3) Locate and copy libraries
            try
                toolboxPath = franka_toolbox_installation_path_get();
                localLibDir = fullfile(toolboxPath, 'libfranka', 'build', 'usr', 'lib');
                
                if exist(localLibDir, 'dir')
                    fprintf('SSHExecutionTool: Syncing libraries from %s...\n', localLibDir);
                    
                    % Copy all .so files to remote lib directory
                    scpLibCmd = sprintf('scp %s -r "%s"/*.so* %s@%s:%s/', ...
                        sshOpts, localLibDir, sshUser, sshHost, remoteLibDir);
                    
                    [status, result] = system(scpLibCmd);
                    if status ~= 0
                        fprintf('SSHExecutionTool Warning: Failed to copy libraries.\n%s\n', result);
                    end
                else
                    fprintf('SSHExecutionTool Warning: Local lib directory not found at %s\n', localLibDir);
                end
            catch ME
                fprintf('SSHExecutionTool Warning: Could not locate toolbox path: %s\n', ME.message);
            end
            
            % 4) Set permissions
            chmodCmd = sprintf('ssh %s %s@%s ''chmod +x %s/%s''', ...
                sshOpts, sshUser, sshHost, remoteDir, fullExeName);
            [status, result] = system(chmodCmd);
            if status ~= 0
                fprintf('SSHExecutionTool: Failed to set executable permissions.\n%s\n', result);
                fprintf('  Command: %s\n', chmodCmd);
                errFlag = true;
                return;
            end
            
            % 5) Run with LD_LIBRARY_PATH
            fprintf('SSHExecutionTool: Starting application...\n');
            
            % We use a PID file strategy to ensure we can capture the PID without hanging
            % waiting for stdout. 
            pidFile = sprintf('%s/app.pid', remoteDir);
            
            % Clean up old PID file
            rmCmd = sprintf('ssh %s %s@%s ''rm -f %s''', ...
                sshOpts, sshUser, sshHost, pidFile);
            system(rmCmd);
            
            % Run command: Background the process and write PID to file. 
            % ssh -f puts ssh into the background after authentication.
            % We use simple single quotes to protect variables from local shell expansion.
            runCmd = sprintf('ssh -f %s %s@%s ''cd %s && export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH && nohup ./%s > /dev/null 2>&1 < /dev/null & echo $! > %s''', ...
                sshOpts, sshUser, sshHost, remoteDir, fullExeName, pidFile);
            
            [status, result] = system(runCmd);
            
            if status ~= 0
                fprintf('SSHExecutionTool: Failed to start application.\n%s\n', result);
                errFlag = true;
                return;
            end
            
            % 6) Capture PID from file
            % Give a small delay to ensure file is written (usually instant)
            pause(0.5);
            
            readPidCmd = sprintf('ssh %s %s@%s ''cat %s''', ...
                sshOpts, sshUser, sshHost, pidFile);
            [status, result] = system(readPidCmd);
            
            result = strtrim(result);
            pid = str2double(result);
            
            if status == 0 && ~isnan(pid) && pid > 0
                this.RemotePID = pid;
                this.IsRunning = true;
                fprintf('SSHExecutionTool: Started application with PID %d\n', pid);
            else
                this.RemotePID = -1;
                this.IsRunning = true; % Assume running even if PID capture failed
                fprintf('SSHExecutionTool: Application started (PID unknown, output: %s)\n', result);
            end
        end
        
        function errFlag = stopApplication(this)
            %STOPAPPLICATION Stop the application on the remote target
            %   Terminates the running application via SSH.
            %   Returns errFlag = false on success, true on failure.
            
            errFlag = false;
            
            % Always attempt to kill, even if we think it's not running,
            % to clear out any zombie processes that might hold the port.
            
            appPath = this.Application;
            [~, exeName, exeExt] = fileparts(appPath);
            fullExeName = char(strcat(exeName, exeExt));
            
            % Force kill potentially multiple instances
            pkillCmd = sprintf('ssh %s %s@%s "pkill -9 -f ''%s'' 2>/dev/null || true"', ...
                this.SSHOptions, this.SSHUser, this.SSHHost, fullExeName);
            [status, ~] = system(pkillCmd);
            
            this.RemotePID = -1;
            this.IsRunning = false;
            
            if status ~= 0
                % pkill returns non-zero if no process found, which is OK
                errFlag = false;
            end
            
            fprintf('SSHExecutionTool: Stopped application (pkill)\n');
            
            % Small delay to allow OS to free ports
            pause(1.0);
        end
        
        function [status, errFlag] = getApplicationStatus(this)
            %GETAPPLICATIONSTATUS Get the current status of the application
            %   Returns the application status and an error flag.
            %   This enables Monitor & Tune functionality in Simulink.
            
            errFlag = false;
            
            if ~this.IsRunning
                status = target.ApplicationStatus.Stopped;
                return;
            end
            
            appPath = this.Application;
            if isempty(appPath)
                status = target.ApplicationStatus.Unknown;
                return;
            end
            
            [~, exeName, exeExt] = fileparts(appPath);
            fullExeName = char(strcat(exeName, exeExt));
            
            % Check if process is running
            if this.RemotePID > 0
                % Check by PID
                checkCmd = sprintf('ssh %s %s@%s "kill -0 %d 2>/dev/null && echo running || echo stopped"', ...
                    this.SSHOptions, this.SSHUser, this.SSHHost, this.RemotePID);
            else
                % Check by name
                checkCmd = sprintf('ssh %s %s@%s "pgrep -f ''%s'' >/dev/null && echo running || echo stopped"', ...
                    this.SSHOptions, this.SSHUser, this.SSHHost, fullExeName);
            end
            
            [exitCode, result] = system(checkCmd);
            result = strtrim(result);
            
            if exitCode ~= 0
                % SSH connection failed
                status = target.ApplicationStatus.Unknown;
                errFlag = true;
                return;
            end
            
            if strcmp(result, 'running')
                status = target.ApplicationStatus.Running;
            else
                status = target.ApplicationStatus.Stopped;
                this.IsRunning = false;
                this.RemotePID = -1;
            end
        end
    end
end
