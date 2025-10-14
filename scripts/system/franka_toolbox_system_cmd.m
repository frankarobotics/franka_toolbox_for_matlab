function [status, cmdout] = franka_toolbox_system_cmd(cmd,path,echo)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    if nargin < 2
        path = '.';
        echo = false;
    end
    
    if nargin < 3
        echo = false;
    end
    
    if isunix()
        if echo
            [status, cmdout] = system(['LD_LIBRARY_PATH="" && pushd "',path,'" && ',cmd, '&& popd'], '-echo');
        else
            [status, cmdout] = system(['LD_LIBRARY_PATH="" && pushd "',path,'" && ',cmd, '&& popd']);
        end
    elseif ispc()
        if echo
            [status, cmdout] = system(['pushd "',path,'" && ',cmd, '&& popd'], '-echo');
        else
            [status, cmdout] = system(['pushd "',path,'" && ',cmd, '&& popd']);
        end
    end
    
end