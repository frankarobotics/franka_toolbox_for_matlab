function franka_toolbox_dist_make()
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    %% Clean-up env
    rm_dir('dist');
    addpath(genpath('../franka_matlab'));
    
    % Get the current directory as project root (since this script is in the root)
    project_root = pwd;
    
    % Check if we're in a git repository before running git clean
    if exist(fullfile(project_root, '.git'), 'dir')
        system(['cd ',project_root,' && git clean -ffxd']);
    else
        fprintf('Not in a git repository, skipping git clean\n');
    end

    %% Copy the Project
    dist_dir = fullfile(project_root, 'dist');
    target_dir = fullfile(dist_dir, 'franka_matlab');
    
    % Create dist directory if it doesn't exist
    if ~exist(dist_dir, 'dir')
        mkdir(dist_dir);
    end
    
    % Copy project files to distribution directory, excluding dist folder
    copy_project_files(project_root, target_dir);

    %% Remove build and other artifacts
    rm_dir(fullfile(target_dir,'build'));
    rm_dir(fullfile(target_dir,'cmake'));
    rm_dir(fullfile(target_dir,'libfranka'));
    rm_dir(fullfile(target_dir,'libfranka_arm'));
    delete(fullfile(target_dir,'.gitignore'));
    delete(fullfile(target_dir,'CHANGELOG.md'));
    delete(fullfile(target_dir,'README.md'));
    delete(fullfile(target_dir,'LICENCE'));
    delete(fullfile(target_dir,'franka_toolbox_dist_make.m'));
    
    remove_all_files_of_type_recursively('.asv',target_dir,{''});

    %% Remove build artifacts (duplicate line removed)

    %% Make the Franka Toolbox for MATLAB
    addpath(genpath(fullfile(project_root,'dist')));
    rmpath(genpath('../franka_matlab'));
    addpath(genpath(fullfile(project_root,'dist')));

    matlab.addons.toolbox.packageToolbox(fullfile(target_dir,'franka_toolbox.prj'),fullfile(project_root,'dist','franka'))

    addpath(genpath('../franka_matlab'));
    rmpath(genpath(fullfile(project_root,'dist')));

end

%% Utilities
function copy_project_files(source_dir, target_dir)
    % Create target directory if it doesn't exist
    if ~exist(target_dir, 'dir')
        mkdir(target_dir);
    end
    
    % Get all items in the source directory
    items = dir(source_dir);
    
    % Remove current and parent directory entries
    items = items(~ismember({items.name}, {'.', '..'}));
    
    % Copy each item, excluding 'dist' folder
    for i = 1:length(items)
        item = items(i);
        if strcmp(item.name, 'dist')
            continue; % Skip the dist directory
        end
        
        source_path = fullfile(source_dir, item.name);
        target_path = fullfile(target_dir, item.name);
        
        % Copy file or directory (copyfile handles both)
        copyfile(source_path, target_path);
    end
end

function rm_dir(dir)
if exist(dir,'dir')
    rmdir(dir,'s');
end
end

function files = find_all_files_of_type_in_directory_recursively(file_type,directory)
     files = dir(fullfile(directory,'**',['*',file_type]));
end

function remove_all_files_of_type_recursively(file_type,directory,white_list)
    src_files = find_all_files_of_type_in_directory_recursively(file_type,directory);
    src_files(ismember({src_files.name}, white_list)) = [];
    arrayfun(@(c) delete(fullfile(c.folder,c.name)), src_files, 'UniformOutput',false);
end