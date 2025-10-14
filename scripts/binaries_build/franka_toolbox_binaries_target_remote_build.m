function franka_toolbox_binaries_target_remote_build(user,ip,port)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    fprintf('Starting remote build process...\n');
    fprintf('Target: %s@%s:%s\n', user, ip, port);

    libfranka_ver = readcell('libfranka_ver.csv');
    libfranka_ver = libfranka_ver{1};
    fprintf('Using libfranka version: %s\n', libfranka_ver);

    %% remote target arm
    % libfranka 
    fprintf('\n=== Building libfranka ===\n');
    franka_toolbox_libfranka_remote_build(user,ip,port,libfranka_ver,true);

    % common 
    fprintf('\n=== Building common components ===\n');
    franka_toolbox_common_build(user,ip,port);

    % FrankaRobot() server
    fprintf('\n=== Building FrankaRobot server ===\n');
    franka_robot_server_build(user,ip,port);
    
    fprintf('\nRemote build completed successfully!\n');
end