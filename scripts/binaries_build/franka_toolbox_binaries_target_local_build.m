function franka_toolbox_binaries_target_local_build()

    libfranka_ver = readcell('libfranka_ver.csv');
    libfranka_ver = libfranka_ver{1};

    if isunix()
        %% local target x86
        % libfranka
        franka_toolbox_libfranka_build(libfranka_ver,true);
    
        % common 
        franka_toolbox_common_build();
    
        % FrankaRobot() server
        franka_robot_server_build();
    end

end