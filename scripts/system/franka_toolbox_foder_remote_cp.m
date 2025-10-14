function [status, cmdout] = franka_toolbox_foder_remote_cp(folder,user,ip,destination,port,verbose)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    if nargin < 6
        verbose = false;
    end
    
    if nargin < 4
        destination = '~/';
        port = '22';
    end

    if nargin < 5
        port = '22';
    end

    scp_cmd = ['scp -o BatchMode=yes -o ConnectTimeout=10 ','-P ',port,' -r ',folder,' ',user,'@',ip,':',destination];
    
    if verbose
        fprintf('Executing remote copy command:\n%s\n', scp_cmd);
    end
    
    [status, cmdout] = franka_toolbox_system_cmd(scp_cmd,'.',verbose);
    
    if verbose
        fprintf('Copy command output:\n%s\n', cmdout);
    end
    
    % Check for errors
    if status ~= 0
        error('Failed to copy folder with status %d:\n%s', status, cmdout);
    end
end