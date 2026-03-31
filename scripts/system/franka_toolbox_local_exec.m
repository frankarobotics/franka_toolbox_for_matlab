function [status, output] = franka_toolbox_local_exec(cmd, path, options)
    %FRANKA_TOOLBOX_LOCAL_EXEC Execute a command locally in a specified directory
    %
    %   [status, output] = franka_toolbox_local_exec(cmd)
    %   [status, output] = franka_toolbox_local_exec(cmd, path)
    %   [status, output] = franka_toolbox_local_exec(cmd, path, options)
    %
    %   Inputs:
    %       cmd      - Command to execute
    %       path     - Directory to execute command in (default: '.')
    %       options  - Struct with optional fields:
    %                  .verbose  - Echo command output (default: false)
    %                  .nothrow  - Don't throw error on failure (default: true)
    %
    %   Outputs:
    %       status   - Exit status of the command (0 = success)
    %       output   - Command output string
    %
    %   Notes:
    %       - On Unix systems, LD_LIBRARY_PATH is cleared to avoid conflicts
    %         with MATLAB's bundled libraries during compilation
    %       - The function uses pushd/popd to change directories safely
    %
    %   Examples:
    %       % Run cmake in build directory
    %       [s, o] = franka_toolbox_local_exec('cmake ..', '/path/to/build');
    %       
    %       % Run with verbose output
    %       opts.verbose = true;
    %       franka_toolbox_local_exec('make', '/path/to/project', opts);
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package
    
    % Default arguments
    if nargin < 2 || isempty(path)
        path = '.';
    end
    
    if nargin < 3
        options = struct();
    end
    
    % Parse options with defaults
    verbose = false;
    nothrow = true;
    
    if isfield(options, 'verbose')
        verbose = options.verbose;
    end
    if isfield(options, 'nothrow')
        nothrow = options.nothrow;
    end
    
    % Build the command with directory change
    if isunix()
        % Clear LD_LIBRARY_PATH to avoid conflicts with MATLAB's bundled libraries
        full_cmd = sprintf('LD_LIBRARY_PATH="" && pushd "%s" && %s && popd', path, cmd);
    else
        % Windows - just change directory
        full_cmd = sprintf('pushd "%s" && %s && popd', path, cmd);
    end
    
    % Execute command
    if verbose
        [status, output] = system(full_cmd, '-echo');
    else
        [status, output] = system(full_cmd);
    end
    
    output = strtrim(output);
    
    % Handle errors
    if status ~= 0 && ~nothrow
        error('FrankaToolbox:LocalExec:Failed', ...
            'Command failed (status %d) in directory: %s\nCommand: %s\nOutput: %s', ...
            status, path, cmd, output);
    end
end

