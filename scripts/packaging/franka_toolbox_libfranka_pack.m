function franka_toolbox_libfranka_pack(remote)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    if nargin == 0
        remote = false;
    end
    
    installation_path = franka_toolbox_installation_path_get();

    dependencies_path = fullfile(installation_path,'dependencies');

    % Copy remote libfranka installation in arm pc
    if remote
        libfranka_path = fullfile(installation_path,'libfranka_arm');
        libfranka_package_path = fullfile(dependencies_path,'libfranka_arm');
    else
        libfranka_path = fullfile(installation_path,'libfranka');
        libfranka_package_path = fullfile(dependencies_path,'libfranka');
    end

    if isfolder(libfranka_package_path), rmdir(libfranka_package_path,'s'); end

    mkdir(libfranka_package_path);

    % Define base components with specific subfolders
    libfranka_components = {
        {fullfile('build', 'usr')}, ...
        {fullfile('common', 'include')}, ...
        'include'  
    };

    % Dynamically find all libfranka.so* files
    build_dir = fullfile(libfranka_path, 'build');
    if exist(build_dir, 'dir')
        so_files = dir(fullfile(build_dir, 'libfranka.so*'));
        num_so_files = length(so_files);
        libfranka_components(end+1:end+num_so_files) = cell(1, num_so_files);
        for i = 1:num_so_files
            libfranka_components{end-num_so_files+i} = {fullfile('build', so_files(i).name)};
        end
    end

    % Packing libfranka 
    for i = 1:length(libfranka_components)
        if iscell(libfranka_components{i})
            % For nested folders, check if source exists before copying
            source_path = fullfile(libfranka_path, libfranka_components{i}{1});
            target_path = fullfile(libfranka_package_path, libfranka_components{i}{1});
            
            if exist(source_path, 'dir')
                % Handle directory copying
                [parent_path, ~] = fileparts(target_path);
                if ~isfolder(parent_path), mkdir(parent_path); end
                
                if ~isempty(dir(fullfile(source_path, '*')))
                    copyfile(source_path, target_path);
                end
            elseif exist(source_path, 'file')
                % Handle file copying
                [parent_path, ~] = fileparts(target_path);
                if ~isfolder(parent_path), mkdir(parent_path); end
                copyfile(source_path, target_path);
            end
        else
            copyfile(fullfile(libfranka_path, libfranka_components{i}), ...
                    fullfile(libfranka_package_path, libfranka_components{i}));
        end
    end

    % Zip libfranka
    if remote
        zip(fullfile(dependencies_path,'libfranka_arm'),libfranka_package_path);
    else
        zip(fullfile(dependencies_path,'libfranka'),libfranka_package_path);
    end
    
    if isfolder(libfranka_package_path), rmdir(libfranka_package_path,'s'); end

end