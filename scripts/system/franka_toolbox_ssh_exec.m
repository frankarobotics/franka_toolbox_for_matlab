function [status, output] = franka_toolbox_ssh_exec(cmd, user, ip, port, options)
    %FRANKA_TOOLBOX_SSH_EXEC Execute a command on a remote machine via SSH
    %
    %   [status, output] = franka_toolbox_ssh_exec(cmd, user, ip)
    %   [status, output] = franka_toolbox_ssh_exec(cmd, user, ip, port)
    %   [status, output] = franka_toolbox_ssh_exec(cmd, user, ip, port, options)
    %
    %   Inputs:
    %       cmd      - Command to execute on remote machine
    %       user     - SSH username
    %       ip       - IP address or hostname of remote machine
    %       port     - SSH port (default: '22')
    %       options  - Struct with optional fields:
    %                  .verbose  - Echo command output (default: false)
    %                  .nothrow  - Don't throw error on failure (default: true)
    %                  .timeout  - Connection timeout in seconds (default: 5)
    %
    %   Outputs:
    %       status   - Exit status of the command (0 = success)
    %       output   - Command output string
    %
    %   Examples:
    %       % Check if file exists (non-throwing)
    %       [s, ~] = franka_toolbox_ssh_exec('test -f /path/file', 'user', '192.168.1.1');
    %       
    %       % Run command and throw on failure
    %       opts.nothrow = false;
    %       franka_toolbox_ssh_exec('ls -la', 'user', '192.168.1.1', '22', opts);
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package
    
    % Default arguments
    if nargin < 4 || isempty(port)
        port = '22';
    end
    
    if nargin < 5
        options = struct();
    end
    
    % Parse options with defaults
    verbose = false;
    nothrow = true;
    timeout = 5;
    
    if isfield(options, 'verbose')
        verbose = options.verbose;
    end
    if isfield(options, 'nothrow')
        nothrow = options.nothrow;
    end
    if isfield(options, 'timeout')
        timeout = options.timeout;
    end
    
    % Build SSH command based on platform
    if ispc()
        % Windows - use OpenSSH
        ssh_exe = 'ssh.exe';
    else
        ssh_exe = 'ssh';
    end
    
    % Build the full SSH command with standard options
    ssh_cmd = sprintf('%s -o ConnectTimeout=%d -o BatchMode=yes -p %s %s@%s "%s"', ...
        ssh_exe, timeout, port, user, ip, cmd);
    
    if verbose
        fprintf('Executing SSH command:\n  %s\n', cmd);
    end
    
    % Execute command
    if verbose
        [status, output] = system(ssh_cmd, '-echo');
    else
        [status, output] = system(ssh_cmd);
    end
    
    output = strtrim(output);
    
    % Handle errors
    if status ~= 0 && ~nothrow
        error('FrankaToolbox:SSHExec:Failed', ...
            'SSH command failed (status %d) on %s@%s:%s\nCommand: %s\nOutput: %s', ...
            status, user, ip, port, cmd, output);
    end
end

