function franka_toolbox_install()
    %FRANKA_TOOLBOX_INSTALL Installs the Franka Toolbox for MATLAB
    %   This function handles the complete installation process of the Franka Toolbox,
    %   including binary unpacking and Simulink configuration.
    %
    %   Copyright (c) 2026 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package

    % Clear command window and display installation start message
    clc;
    fprintf('\nInitiating Franka Toolbox installation...\n\n');

    % Perform installation steps
    try
        % Remove any existing installation
        franka_toolbox_uninstall();

        binaryGroups = getBinaryGroups();
        if hasMissingBinaries(binaryGroups)
            warning('franka_toolbox_install:MissingBinaries', '%s', ...
                    getMissingBinariesWarningMessage(binaryGroups));
            fprintf('\nFranka Toolbox installation unsuccessful or partially incomplete\n\n');
            return;
        end

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
        franka_simulink_library = fullfile(installation_path, 'franka_simulink_library');
        tryUnzip(fullfile(franka_simulink_library,'bin.zip'), ...
                 fullfile(franka_simulink_library,'blocks'));
        
        % Unpack common binaries
        tryUnzip(fullfile(installation_path, 'common', 'bin.zip'), ...
                 fullfile(installation_path, 'common'));
        tryUnzip(fullfile(installation_path, 'common', 'bin_arm.zip'), ...
                 fullfile(installation_path, 'common'), false);
        
        % Unpack MATLAB library binaries
        matlab_robot_server_path = fullfile(installation_path, 'franka_robot_server');
        tryUntar(fullfile(matlab_robot_server_path, 'bin.tar.gz'), matlab_robot_server_path);
        tryUntar(fullfile(matlab_robot_server_path, 'bin_arm.tar.gz'), matlab_robot_server_path, false);
        
        matlab_lib_path = fullfile(installation_path, 'franka_robot');
        tryUnzip(fullfile(matlab_lib_path, 'bin.zip'), matlab_lib_path);
        matlab_bin_path = fullfile(matlab_lib_path, 'bin');
        if ~isfolder(matlab_bin_path)
            error('Required MATLAB binaries could not be unpacked from: %s', ...
                  fullfile(matlab_lib_path, 'bin.zip'));
        end
        addpath(matlab_bin_path);
        
        % Unpack dependencies
        deps_path = fullfile(installation_path, 'dependencies');
        tryUnzip(fullfile(deps_path, 'libfranka.zip'), installation_path);
        tryUnzip(fullfile(deps_path, 'libfranka_arm.zip'), installation_path, false);
    end

    function tryUnzip(archivePath, destPath, warnIfMissing)
        % Attempts to unzip an archive, optionally warns if file not found
        if nargin < 3
            warnIfMissing = true;
        end

        if ~isfile(archivePath)
            if warnIfMissing
                warning('Archive not found, skipping: %s', archivePath);
            end
            return;
        end
        try
            unzip(archivePath, destPath);
        catch ME
            error('Failed to unzip %s: %s', archivePath, ME.message);
        end
    end

    function tryUntar(archivePath, destPath, warnIfMissing)
        % Attempts to untar an archive, optionally warns if file not found
        if nargin < 3
            warnIfMissing = true;
        end

        if ~isfile(archivePath)
            if warnIfMissing
                warning('Archive not found, skipping: %s', archivePath);
            end
            return;
        end
        try
            untar(archivePath, destPath);
        catch ME
            error('Failed to untar %s: %s', archivePath, ME.message);
        end
    end

    function binaryGroups = getBinaryGroups()
        installation_path = franka_toolbox_installation_path_get();
        binaryGroups = struct( ...
            'missingLabel', { ...
                'Missing MATLAB host archives', ...
                'Missing x86_64 target archives (bin)', ...
                'Missing ARM64 target archives (bin_arm)' ...
            }, ...
            'foundLabel', { ...
                'Found MATLAB host archives', ...
                'Found x86_64 target archives (bin)', ...
                'Found ARM64 target archives (bin_arm)' ...
            }, ...
            'requiredPaths', { ...
                { ...
                    fullfile(installation_path, 'franka_simulink_library', 'bin.zip'), ...
                    fullfile(installation_path, 'franka_robot', 'bin.zip') ...
                }, ...
                { ...
                    fullfile(installation_path, 'common', 'bin.zip'), ...
                    fullfile(installation_path, 'franka_robot_server', 'bin.tar.gz'), ...
                    fullfile(installation_path, 'dependencies', 'libfranka.zip') ...
                }, ...
                { ...
                    fullfile(installation_path, 'common', 'bin_arm.zip'), ...
                    fullfile(installation_path, 'franka_robot_server', 'bin_arm.tar.gz'), ...
                    fullfile(installation_path, 'dependencies', 'libfranka_arm.zip') ...
                } ...
            } ...
        );

        for i = 1:numel(binaryGroups)
            [binaryGroups(i).missingPaths, binaryGroups(i).foundPaths] = ...
                splitArchivesByAvailability(binaryGroups(i).requiredPaths);
        end
    end

    function [missingArchives, foundArchives] = splitArchivesByAvailability(requiredArchives)
        missingMask = ~cellfun(@isfile, requiredArchives);
        missingArchives = requiredArchives(missingMask);
        foundArchives = requiredArchives(~missingMask);
    end

    function tf = hasMissingBinaries(binaryGroups)
        tf = any(arrayfun(@(group) ~isempty(group.missingPaths), binaryGroups));
    end

    function message = getMissingBinariesWarningMessage(binaryGroups)
        installation_path = franka_toolbox_installation_path_get();
        missingSections = cell(0, 1);
        foundSections = cell(0, 1);
        for i = 1:numel(binaryGroups)
            if ~isempty(binaryGroups(i).missingPaths)
                relativeMissingArchives = cellfun( ...
                    @(path) strrep(path, [installation_path filesep], ''), ...
                    binaryGroups(i).missingPaths, 'UniformOutput', false);
                missingSections{end + 1} = sprintf('%s:\n - %s\n', ...
                    binaryGroups(i).missingLabel, ...
                    strjoin(relativeMissingArchives, '\n - ')); %#ok<AGROW>
            end

            if isempty(binaryGroups(i).foundPaths)
                continue;
            end
            relativeFoundArchives = cellfun( ...
                @(path) strrep(path, [installation_path filesep], ''), ...
                binaryGroups(i).foundPaths, 'UniformOutput', false);
            foundSections{end + 1} = sprintf('%s:\n - %s\n', ...
                binaryGroups(i).foundLabel, ...
                strjoin(relativeFoundArchives, '\n - ')); %#ok<AGROW>
        end
        missingList = strjoin(missingSections, '\n');
        if isempty(foundSections)
            foundList = ' - none';
        else
            foundList = strjoin(foundSections, '\n');
        end

        message = sprintf([ ...
            ['One or more binaries were not found. The Toolbox won''t function partially or at all ' ...
             'depending on the binaries found (check warning message below).\n\n'] ...
            '%s\n' ...
            'Found archives:\n%s\n\n' ...
            'Please\n\n' ...
            '1. In case the official mtlbx release has been installed and the source code has been cloned please make sure that the source code is not found in path.\n' ...
            '2. If you''ve cloned the source code please make sure you''ve built the missing x86_64 and/or ARM64 binaries with ./build.sh [amd64|arm64] --libfranka <version>, and then run franka_robot_mex() and franka_simulink_library_mex().' ...
        ], missingList, foundList);
    end

    function configureSimulink()
        % Refresh Simulink Library Browser
        sl_refresh_customizations();
        
        % Configure libfranka installation (default: local)
        franka_toolbox_libfranka_system_installation_set(false);
    end
end