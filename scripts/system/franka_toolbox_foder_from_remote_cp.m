function [status, cmdout] = franka_toolbox_foder_from_remote_cp(folder,destination_path,user,ip,port,verbose)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    if nargin < 5
        port = '22';
    end

    if nargin < 6
        verbose = false;
    end

    if verbose
        [status, cmdout] = franka_toolbox_system_cmd(['scp -o BatchMode=yes -o ConnectTimeout=10 ','-P ',port,' -r ',user,'@',ip,':',folder,' ', destination_path],'.',true);
    else
        [status, cmdout] = franka_toolbox_system_cmd(['scp -o BatchMode=yes -o ConnectTimeout=10 ','-P ',port,' -r ',user,'@',ip,':',folder,' ', destination_path],'.',false);
    end
    
    % Check for errors
    if status ~= 0
        error('Failed to copy folder from remote with status %d:\n%s', status, cmdout);
    end
end