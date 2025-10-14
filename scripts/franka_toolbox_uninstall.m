function franka_toolbox_uninstall()
%  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
%  This file is subject to the terms and conditions defined in the file
%  'LICENSE' , which is part of this package
clc;
fprintf('\nRemoving the Franka Toolbox for MATLAB local artifacts...\n');

if ~isempty(getpref('franka_toolbox'))
    rmpref('franka_toolbox');
end

fprintf('Franka Toolbox for MATLAB local infos have been succesfully removed. \n');