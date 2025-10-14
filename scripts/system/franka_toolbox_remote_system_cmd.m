function [s, r] = franka_toolbox_remote_system_cmd(cmd, path, user, ip, port, verbose)
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    if nargin < 6
        verbose = false;
    end
    
    if nargin < 5
        port = '22';
    end
    
    % Convert path to use forward slashes for SSH compatibility
    path = strrep(path, '\', '/');
    
    % Check if running on Windows
    if ispc
        % Use full path to OpenSSH if it exists in Windows
        ssh_cmd = 'C:\Windows\System32\OpenSSH\ssh.exe';
        if ~exist(ssh_cmd, 'file')
            error('OpenSSH not found. Please install OpenSSH for Windows or add it to System32/OpenSSH.');
        end
    else
        ssh_cmd = 'ssh';
    end
    
    full_cmd = ['LD_LIBRARY_PATH='''' && cd "', path, '" && ', cmd];
    ssh_cmd_full = ['"', ssh_cmd, '" -o BatchMode=yes -o ConnectTimeout=10 ', user, '@', ip, ' -p ', port, ' "', full_cmd, '"'];
    
    if verbose
        fprintf('Executing remote command:\n%s\n', full_cmd);
    end
    
    if verbose
        [s, r] = system(ssh_cmd_full, '-echo');
    else
        [s, r] = system(ssh_cmd_full);
    end
    
    % Check for errors
    if s ~= 0
        error('Remote command failed with status %d on %s@%s:%s\nCommand: %s\nOutput: %s', s, user, ip, port, full_cmd, r);
    end
end