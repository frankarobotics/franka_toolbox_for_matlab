function franka_toolbox_dist_make(options)
    %FRANKA_TOOLBOX_DIST_MAKE Create distribution package for Franka Toolbox
    %
    %   franka_toolbox_dist_make() - Full build with git clean (for local dev)
    %   franka_toolbox_dist_make('ci') - CI mode: skip git clean (for GitHub Actions)
    %
    %   Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE' , which is part of this package
    
    arguments
        options.Mode {mustBeMember(options.Mode, {'local', 'ci'})} = 'local'
    end
    
    ci_mode = strcmp(options.Mode, 'ci');
    
    %% Clean-up env
    rm_dir('dist');
    
    % Add parent franka_matlab if it exists (for local development)
    parent_franka_matlab = '../franka_matlab';
    has_parent_franka = exist(parent_franka_matlab, 'dir');
    if has_parent_franka
        addpath(genpath(parent_franka_matlab));
    end
    
    % Get the current directory as project root (since this script is in the root)
    project_root = pwd;
    
    % Check if we're in a git repository before running git clean
    % Skip in CI mode to preserve downloaded artifacts
    if ~ci_mode && exist(fullfile(project_root, '.git'), 'dir')
        fprintf('Running git clean (local mode)...\n');
        system(['cd ',project_root,' && git clean -ffxd']);
    else
        if ci_mode
            fprintf('CI mode: skipping git clean to preserve artifacts\n');
        else
            fprintf('Not in a git repository, skipping git clean\n');
        end
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
    rm_dir(fullfile(target_dir,'docker'));
    rm_dir(fullfile(target_dir,'.github'));
    safe_delete(fullfile(target_dir,'.gitignore'));
    safe_delete(fullfile(target_dir,'CHANGELOG.md'));
    safe_delete(fullfile(target_dir,'README.md'));
    safe_delete(fullfile(target_dir,'LICENCE'));
    safe_delete(fullfile(target_dir,'LICENSE'));
    safe_delete(fullfile(target_dir,'franka_toolbox_dist_make.m'));
    
    remove_all_files_of_type_recursively('.asv',target_dir,{''});

    %% Make the Franka Toolbox for MATLAB
    addpath(genpath(fullfile(project_root,'dist')));
    if has_parent_franka
        rmpath(genpath(parent_franka_matlab));
    end
    addpath(genpath(fullfile(project_root,'dist')));

    fprintf('Packaging toolbox...\n');
    matlab.addons.toolbox.packageToolbox(fullfile(target_dir,'franka_toolbox.prj'),fullfile(project_root,'dist','franka'))
    fprintf('Toolbox packaged successfully: dist/franka.mltbx\n');

    if has_parent_franka
        addpath(genpath(parent_franka_matlab));
    end
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

function safe_delete(filepath)
if exist(filepath,'file')
    delete(filepath);
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