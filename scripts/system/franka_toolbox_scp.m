function [status, output] = franka_toolbox_scp(source, destination, user, ip, port, options)
    %FRANKA_TOOLBOX_SCP Copy files/folders to or from a remote machine via SCP
    %
    %   [status, output] = franka_toolbox_scp(source, destination, user, ip)
    %   [status, output] = franka_toolbox_scp(source, destination, user, ip, port)
    %   [status, output] = franka_toolbox_scp(source, destination, user, ip, port, options)
    %
    %   Inputs:
    %       source      - Source path (local or remote with ':' prefix for remote)
    %       destination - Destination path (local or remote with ':' prefix for remote)
    %       user        - SSH username
    %       ip          - IP address or hostname of remote machine
    %       port        - SSH port (default: '22')
    %       options     - Struct with optional fields:
    %                     .recursive - Copy directories recursively (default: false)
    %                     .verbose   - Echo command output (default: false)
    %                     .nothrow   - Don't throw error on failure (default: false)
    %                     .timeout   - Connection timeout in seconds (default: 10)
    %
    %   For remote paths, use ':' prefix to indicate remote:
    %       ':remote/path' means the path is on the remote machine
    %       'local/path'   means the path is on the local machine
    %
    %   Examples:
    %       % Copy local file to remote
    %       franka_toolbox_scp('/local/file.txt', ':/remote/dir/', 'user', '192.168.1.1');
    %       
    %       % Copy remote folder to local (recursive)
    %       opts.recursive = true;
    %       franka_toolbox_scp(':/remote/folder', '/local/dest/', 'user', '192.168.1.1', '22', opts);
    %
    %       % Copy local folder to remote (recursive, non-throwing)
    %       opts.recursive = true;
    %       opts.nothrow = true;
    %       [s, o] = franka_toolbox_scp('/local/folder', ':/remote/', 'user', '192.168.1.1', '22', opts);
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package
    
    % Default arguments
    if nargin < 5 || isempty(port)
        port = '22';
    end
    
    if nargin < 6
        options = struct();
    end
    
    % Parse options with defaults
    recursive = false;
    verbose = false;
    nothrow = false;
    timeout = 10;
    
    if isfield(options, 'recursive')
        recursive = options.recursive;
    end
    if isfield(options, 'verbose')
        verbose = options.verbose;
    end
    if isfield(options, 'nothrow')
        nothrow = options.nothrow;
    end
    if isfield(options, 'timeout')
        timeout = options.timeout;
    end
    
    % Build SCP command based on platform
    if ispc()
        scp_exe = 'scp.exe';
    else
        scp_exe = 'scp';
    end
    
    % Build flags
    flags = sprintf('-o ConnectTimeout=%d -o BatchMode=yes -P %s', timeout, port);
    if recursive
        flags = [flags ' -r'];
    end
    
    % Process source and destination paths
    source_path = processPath(source, user, ip);
    dest_path = processPath(destination, user, ip);
    
    % Quote paths to handle spaces
    source_path = quotePath(source_path);
    dest_path = quotePath(dest_path);
    
    % Build full SCP command
    scp_cmd = sprintf('%s %s %s %s', scp_exe, flags, source_path, dest_path);
    
    if verbose
        fprintf('Executing SCP command:\n  %s\n', scp_cmd);
    end
    
    % Execute command
    if verbose
        [status, output] = system(scp_cmd, '-echo');
    else
        [status, output] = system(scp_cmd);
    end
    
    output = strtrim(output);
    
    % Handle errors
    if status ~= 0 && ~nothrow
        error('FrankaToolbox:SCP:Failed', ...
            'SCP command failed (status %d)\nCommand: %s\nOutput: %s', ...
            status, scp_cmd, output);
    end
end

function path = processPath(inputPath, user, ip)
    % Convert path notation: ':path' means remote, 'path' means local
    if startsWith(inputPath, ':')
        % Remote path - prepend user@ip:
        remotePath = inputPath(2:end); % Remove leading ':'
        remotePath = strrep(remotePath, '\', '/'); % Normalize path separators
        path = sprintf('%s@%s:%s', user, ip, remotePath);
    else
        % Local path - normalize separators for command line
        path = strrep(inputPath, '\', '/');
    end
end

function quotedPath = quotePath(path)
    % Quote path to handle spaces and special characters
    % Don't double-quote if already quoted
    if ~startsWith(path, '"')
        quotedPath = ['"' path '"'];
    else
        quotedPath = path;
    end
end

