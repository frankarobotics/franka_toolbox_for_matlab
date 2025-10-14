function franka_toolbox_install()
    %FRANKA_TOOLBOX_INSTALL Installs the Franka Toolbox for MATLAB
    %   This function handles the complete installation process of the Franka Toolbox,
    %   including binary unpacking and Simulink configuration.
    %
    %   Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package

    % Clear command window and display installation start message
    clc;
    fprintf('\nInitiating Franka Toolbox installation...\n\n');

    % Perform installation steps
    try
        % Remove any existing installation
        franka_toolbox_uninstall();

        % Extract binary files
        unpackBinaries();

        % Configure Simulink environment
        configureSimulink();

        % Success message
        fprintf('\nFranka Toolbox installation completed successfully!\n\n');
    catch ME
        fprintf('\nInstallation failed: %s\n', ME.message);
        rethrow(ME);
    end

    %% Helper Functions
    function unpackBinaries()
        installation_path = franka_toolbox_installation_path_get();

        % Unzip Simulink binaries
        franka_toolbox_simulink_library = fullfile(installation_path, 'franka_toolbox_simulink_library');
        unzip(fullfile(franka_toolbox_simulink_library,'bin.zip'), ...
              fullfile(franka_toolbox_simulink_library,'blocks'));
        
        % Unpack common binaries
        unzip(fullfile(installation_path, 'common', 'bin.zip'), ...
              fullfile(installation_path, 'common'));
        unzip(fullfile(installation_path, 'common', 'bin_arm.zip'), ...
              fullfile(installation_path, 'common'));
        
        % Unpack MATLAB library binaries
        matlab_robot_server_path = fullfile(installation_path, 'franka_robot_server');
        untar(fullfile(matlab_robot_server_path, 'bin.tar.gz'), matlab_robot_server_path);
        untar(fullfile(matlab_robot_server_path, 'bin_arm.tar.gz'), matlab_robot_server_path);
        
        matlab_lib_path = fullfile(installation_path, 'franka_robot');
        unzip(fullfile(matlab_lib_path, 'bin.zip'), matlab_lib_path);
        addpath(fullfile(matlab_lib_path, 'bin'));
        
        % Unpack dependencies
        deps_path = fullfile(installation_path, 'dependencies');
        unzip(fullfile(deps_path, 'libfranka.zip'), installation_path);
        unzip(fullfile(deps_path, 'libfranka_arm.zip'), installation_path);
    end

    function configureSimulink()
        % Refresh Simulink Library Browser
        sl_refresh_customizations();
        
        % Configure libfranka installation (default: local)
        franka_toolbox_libfranka_system_installation_set(false);
    end
end