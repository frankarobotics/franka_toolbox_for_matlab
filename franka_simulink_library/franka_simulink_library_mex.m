function franka_simulink_library_mex()
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    % Get the installation path
    installation_path = franka_toolbox_installation_path_get();
    
    % Set paths
    simulink_lib_path = fullfile(installation_path, 'franka_simulink_library');
    build_dir = fullfile(simulink_lib_path, 'build');
    target_dir = fullfile(simulink_lib_path, 'bin');
    bin_zip = fullfile(simulink_lib_path, 'bin.zip');
    
    % Create bin directory if it doesn't exist (don't remove if it exists)
    if ~exist(target_dir, 'dir')
        mkdir(target_dir);
    end
    
    % Handle build directory setup
    if ~exist(build_dir, 'dir')
        mkdir(build_dir);
    end
    
    % Store current directory for later restoration
    current_dir = pwd;
    
    % Change to build directory
    cd(build_dir);
    
    try
        % Configure CMake with verbose output
        disp('Configuring CMake...');
        opts = struct('nothrow', false);
        if ispc
            % For Windows, specify Visual Studio generator with verbose output
            [~, output] = franka_toolbox_local_exec('cmake -G "Visual Studio 17 2022" -A x64 ..', build_dir, opts);
            disp(output);
            
            % Build using CMake with verbose output
            disp('Building library...');
            [~, output] = franka_toolbox_local_exec('cmake --build . --config Release --verbose', build_dir, opts);
            disp(output);
        else
            % For Unix systems (Linux/macOS)
            [~, output] = franka_toolbox_local_exec('cmake ..', build_dir, opts);
            disp(output);
            
            % Build using CMake with verbose output
            disp('Building library...');
            [~, output] = franka_toolbox_local_exec('cmake --build . --verbose', build_dir, opts);
            disp(output);
        end
        
        % Copy MEX files to blocks directory if they're in a different location
        if ispc
            mex_files = dir(fullfile(build_dir, 'Release', '*.mex*'));
        else
            mex_files = dir(fullfile(build_dir, '*.mex*'));
        end
        
        % Copy MEX files
        for i = 1:length(mex_files)
            if ispc
                source_file = fullfile(build_dir, 'Release', mex_files(i).name);
            else
                source_file = fullfile(build_dir, mex_files(i).name);
            end
            copyfile(source_file, fullfile(target_dir, mex_files(i).name));
        end
        
        % After successful build and file copying, update the bin.zip folder
        try
            % Create a temporary directory for merging
            temp_dir = fullfile(simulink_lib_path, 'temp_bin');
            if ~exist(temp_dir, 'dir')
                mkdir(temp_dir);
            end
            
            % If bin.zip exists, extract its contents to temp directory
            if exist(bin_zip, 'file')
                unzip(bin_zip, temp_dir);
            end
            
            % Copy new artifacts from bin directory to temp directory (overwriting if necessary)
            copyfile(fullfile(target_dir, '*'), temp_dir);
            
            % Change to temp directory to zip its contents
            cd(temp_dir);
            
            % Create zip with the merged contents (overwriting existing zip)
            zip(bin_zip, '*');
            
            % Change back to simulink library path
            cd(simulink_lib_path);
            
            % Clean up temporary and bin directories
            rmdir(temp_dir, 's');
            rmdir(target_dir, 's');
            disp('Bin folder successfully merged, zipped and removed.');
            
            % Return to original directory before removing build folder
            cd(current_dir);
            
            % Remove build directory
            if exist(build_dir, 'dir')
                rmdir(build_dir, 's');
                disp('Build directory removed successfully.');
            end
            
        catch ME
            cd(current_dir);
            rethrow(ME);
        end
        
    catch ME
        cd(current_dir);
        rethrow(ME);
    end
end
