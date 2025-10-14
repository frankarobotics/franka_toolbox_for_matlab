function libfranka_version = franka_toolbox_libfranka_remote_build_check(username, ip, port, workspace_dir)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    % Input validation
    arguments
        username char
        ip char
        port char = '22'
        workspace_dir char = '~'
    end
    
    libfranka_version = '';
    
    % Construct base SSH command
    base_ssh_cmd = sprintf('ssh -p %s %s@%s', port, username, ip);
    
    % Check only workspace installation
    build_dir = fullfile(workspace_dir, 'libfranka/build');

    try
        % List all files in the build directory
        cmd = sprintf('%s "ls -l %s 2>/dev/null"', base_ssh_cmd, build_dir);
        [status, result] = system(cmd);
        
        if status == 0
            % Extract version number using regular expression
            version_match = regexp(result, '\d+\.\d+\.\d+', 'match');
            
            if ~isempty(version_match)
                libfranka_version = version_match{1};
            end
        end
    catch
        warning('Failed to check libfranka installation on remote machine');
    end
end 