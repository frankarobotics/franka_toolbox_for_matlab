classdef FrankaRobotServer < handle
    %FRANKAROBOTSERVER Manages franka_robot_server process lifecycle
    
    properties (Access = private)
        pid
        logFile
        outputFid
        Username
        ServerIP = 'localhost'
        SSHPort = '22'
        ServerPort = '5001'
        execDir
        isRemote
        isWindows
        archSuffix
    end
    
    methods
        function obj = FrankaRobotServer(Username, ServerIP, SSHPort, ServerPort)
            obj.isRemote = false;
            obj.isWindows = ispc();
            obj.archSuffix = '';
            
            if nargin == 4
                obj.isRemote = true;
                obj.Username = Username;
                obj.ServerIP = ServerIP;
                obj.SSHPort = SSHPort;
                obj.ServerPort = ServerPort;
                obj.archSuffix = obj.detectRemoteArch();
            end
            
            obj.execDir = fullfile(franka_toolbox_installation_path_get(), ...
                'franka_robot_server', ['bin', obj.archSuffix]);
        end
        
        function start(obj)
            % Check if already running
            if obj.isRunning()
                warning('FrankaRobotServer:AlreadyRunning', ...
                    'Server already running on port %s', obj.ServerPort);
                return;
            end
            
            % Clean up any orphaned process on same port
            obj.cleanupOrphan();
            
            execPath = fullfile(obj.execDir, 'franka_robot_server');
            pidFile = obj.pidPath();
            obj.logFile = obj.logPath();
            
            if obj.isRemote
                remoteDir = obj.remoteDir();
                [s, ~] = obj.ssh(['test -x ' remoteDir '/franka_robot_server']);
                if s ~= 0
                    obj.setupRemoteWorkspace();
                end
                
                esc = ternary(obj.isWindows, '$!', '\$!');
                cmd = sprintf('cd %s && (./franka_robot_server %s %s > %s 2>&1 & echo %s > %s)', ...
                    remoteDir, obj.ServerIP, obj.ServerPort, obj.logFile, esc, pidFile);
                [s, ~] = obj.ssh(cmd);
                if s ~= 0
                    error('FrankaRobotServer:StartFailed', 'Failed to start server');
                end
                pause(0.5);
                [~, pidStr] = obj.ssh(['cat ' pidFile]);
                obj.pid = str2double(strtrim(pidStr));
            else
                if obj.isWindows
                    error('FrankaRobotServer:Unsupported', 'Local mode not supported on Windows');
                end
                if ~exist(execPath, 'file')
                    error('FrankaRobotServer:NotFound', ...
                        'Executable not found. Run franka_robot_server_build()');
                end
                
                q = @(p) ['''' p ''''];
                cmd = sprintf('%s %s %s > %s 2>&1 & echo $! > %s', ...
                    q(execPath), obj.ServerIP, obj.ServerPort, q(obj.logFile), q(pidFile));
                [s, ~] = franka_toolbox_local_exec(cmd, obj.execDir);
                if s ~= 0
                    error('FrankaRobotServer:StartFailed', 'Failed to start server');
                end
                pause(0.5);
                obj.pid = str2double(fileread(pidFile));
                obj.outputFid = fopen(obj.logFile, 'r');
            end
            
            if isempty(obj.pid) || obj.pid <= 0 || isnan(obj.pid)
                error('FrankaRobotServer:StartFailed', 'Failed to get server PID');
            end
        end
        
        function stop(obj)
            if obj.isRunning()
                % Use pgrep + xargs kill (avoids $() which gets expanded locally for SSH)
                % Use [f] trick so pgrep doesn't match itself
                pattern = sprintf('[f]ranka_robot_server.*%s', obj.ServerPort);
                killCmd = sprintf('pgrep -f ''%s'' | xargs kill 2>/dev/null', pattern);
                if obj.isRemote
                    obj.ssh(killCmd);
                else
                    system(killCmd);
                end
                pause(0.3);
            end
            obj.cleanup();
        end
        
        function running = isRunning(obj)
            % Use pgrep to find process by name and port (more robust than PID)
            % Use [f] trick so pgrep doesn't match itself
            pattern = sprintf('[f]ranka_robot_server.*%s', obj.ServerPort);
            if obj.isRemote
                [s, ~] = obj.ssh(sprintf('pgrep -f ''%s'' > /dev/null 2>&1', pattern));
            else
                [s, ~] = system(sprintf('pgrep -f ''%s'' > /dev/null 2>&1', pattern));
            end
            running = (s == 0);
        end
        
        function lines = getOutput(obj)
            lines = {};
            if obj.isRemote
                [~, out] = obj.ssh(['cat ' obj.logFile ' 2>/dev/null']);
                if ~isempty(out)
                    lines = strsplit(out, '\n');
                end
            elseif ~isempty(obj.outputFid) && ~feof(obj.outputFid)
                while ~feof(obj.outputFid)
                    line = fgetl(obj.outputFid);
                    if ischar(line)
                        lines{end+1} = line; %#ok<AGROW>
                    end
                end
            end
        end
        
        function delete(obj)
            obj.stop();
        end
        
        function setupRemoteWorkspace(obj)
            remoteDir = obj.remoteDir();
            remoteMatlabWs = obj.remoteMatlabWsDir();
            scpOpts = struct('recursive', false, 'nothrow', false);
            scpOptsR = struct('recursive', true, 'nothrow', false);
            
            obj.ssh(['mkdir -p ' remoteDir]);
            obj.ssh(['mkdir -p ' remoteMatlabWs]);
            
            execPath = fullfile(obj.execDir, 'franka_robot_server');
            franka_toolbox_scp(execPath, [':' remoteDir '/'], ...
                obj.Username, obj.ServerIP, obj.SSHPort, scpOpts);
            
            libfrankaPath = fullfile(franka_toolbox_installation_path_get(), ...
                ['libfranka' obj.archSuffix], 'build', 'usr');
            franka_toolbox_scp(libfrankaPath, [':' remoteMatlabWs '/'], ...
                obj.Username, obj.ServerIP, obj.SSHPort, scpOptsR);
        end

        function deleteRemoteWorkspace(obj)
            if obj.isRemote
                obj.stop();
                obj.ssh(['rm -rf ' obj.remoteWsRoot()]);
            end
        end

        function value = getUsername(obj), value = obj.Username; end
        function value = getServerIp(obj), value = obj.ServerIP; end
        function value = getSshPort(obj), value = obj.SSHPort; end
        function value = getServerPort(obj), value = obj.ServerPort; end
        
        function setServerPort(obj, value)
            obj.ServerPort = char(value);
        end
    end
    
    methods (Access = private)
        function path = pidPath(obj)
            if obj.isRemote
                path = sprintf('%s/franka_server_%s.pid', obj.remoteDir(), obj.ServerPort);
            else
                path = fullfile(obj.execDir, sprintf('franka_server_%s.pid', obj.ServerPort));
            end
        end
        
        function path = logPath(obj)
            if obj.isRemote
                path = sprintf('%s/franka_server_%s.log', obj.remoteDir(), obj.ServerPort);
            else
                path = fullfile(obj.execDir, sprintf('franka_server_%s.log', obj.ServerPort));
            end
        end
        
        function cleanupOrphan(obj)
            % Check for stale PID file and kill if it's our process
            pidFile = obj.pidPath();
            if obj.isRemote
                [s, pidStr] = obj.ssh(['cat ' pidFile ' 2>/dev/null']);
                if s == 0 && ~isempty(pidStr)
                    stalePid = str2double(strtrim(pidStr));
                    if ~isnan(stalePid) && obj.isOurProcess(stalePid)
                        obj.ssh(sprintf('kill -9 %d 2>/dev/null', stalePid));
                        pause(0.2);
                    end
                end
            else
                if exist(pidFile, 'file')
                    stalePid = str2double(fileread(pidFile));
                    if ~isnan(stalePid) && obj.isOurProcess(stalePid)
                        system(sprintf('kill -9 %d 2>/dev/null', stalePid));
                        pause(0.2);
                    end
                end
            end
        end
        
        function isOurs = isOurProcess(obj, pid)
            % Check if PID belongs to franka_robot_server
            if obj.isRemote
                [s, out] = obj.ssh(sprintf('cat /proc/%d/cmdline 2>/dev/null', pid));
                isOurs = (s == 0) && contains(out, 'franka_robot_server');
            else
                cmdFile = sprintf('/proc/%d/cmdline', pid);
                if exist(cmdFile, 'file')
                    isOurs = contains(fileread(cmdFile), 'franka_robot_server');
                else
                    isOurs = false;
                end
            end
        end
        
        function cleanup(obj)
            pidFile = obj.pidPath();
            if obj.isRemote
                obj.ssh(sprintf('rm -f %s %s', pidFile, obj.logFile));
            else
                if ~isempty(obj.outputFid)
                    fclose(obj.outputFid);
                    obj.outputFid = [];
                end
                if exist(pidFile, 'file'), delete(pidFile); end
                if exist(obj.logFile, 'file'), delete(obj.logFile); end
            end
            obj.pid = [];
        end
        
        function [status, output] = ssh(obj, cmd)
            [status, output] = franka_toolbox_ssh_exec(cmd, obj.Username, ...
                obj.ServerIP, obj.SSHPort);
        end
        
        function arch = detectRemoteArch(obj)
            % Validate connection
            opts = struct('nothrow', false, 'timeout', 5);
            franka_toolbox_ssh_exec('echo ok', obj.Username, obj.ServerIP, obj.SSHPort, opts);
            
            [~, out] = obj.ssh('uname -m');
            out = strtrim(out);
            if any(strcmp(out, {'aarch64', 'arm64', 'armv8l'}))
                arch = '_arm';
            else
                arch = '';
            end
        end
        
        function path = remoteDir(obj)
            if obj.isWindows
                path = ['/home/' obj.Username '/franka_matlab_ws/franka_robot_server/bin' obj.archSuffix];
            else
                path = ['~/franka_matlab_ws/franka_robot_server/bin' obj.archSuffix];
            end
        end
        
        function path = remoteMatlabWsDir(obj)
            if obj.isWindows
                path = ['/home/' obj.Username '/franka_matlab_ws/libfranka' obj.archSuffix '/build'];
            else
                path = ['~/franka_matlab_ws/libfranka' obj.archSuffix '/build'];
            end
        end
        
        function path = remoteWsRoot(obj)
            if obj.isWindows
                path = ['/home/' obj.Username '/franka_matlab_ws'];
            else
                path = '~/franka_matlab_ws';
            end
        end
    end
    
    methods (Static)
        function cleanupRemoteWorkspace(username, serverIP, sshPort)
            if ispc()
                root = ['/home/' username '/franka_matlab_ws'];
            else
                root = '~/franka_matlab_ws';
            end
            franka_toolbox_ssh_exec(['rm -rf ' root], username, serverIP, sshPort);
        end
    end
end

function result = ternary(cond, t, f)
    if cond, result = t; else, result = f; end
end
