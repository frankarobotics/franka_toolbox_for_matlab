function makecfg(objBuildInfo)
    % Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    % This file is subject to the terms and conditions defined in the file
    % 'LICENSE', which is part of this package

    % Get active configuration
    configSet = getActiveConfigSet(gcs);

    % NVIDIA Jetson specific configuration
    if strcmp(get_param(configSet, 'HardwareBoard'), 'NVIDIA Jetson')
        % Add include paths
        addIncludePaths(objBuildInfo, {...
            fullfile(franka_toolbox_installation_path_get(), 'common', 'include'), ...
            fullfile(franka_toolbox_installation_path_get(), 'libfranka_arm', 'include'), ...
            fullfile(franka_toolbox_installation_path_get(), 'libfranka_arm', 'common', 'include')});
        
        % Add link objects
        addLinkObjects(objBuildInfo, ...
            {'libfranka_matlab.a'}, ...
            {fullfile(franka_toolbox_installation_path_get(), 'common', 'bin_arm')}, ...
            1000, true, true);

        % Handle installation path for Windows
        installation_path = franka_toolbox_installation_path_get();
        if ispc()
            installation_path = strrep(installation_path, ':', '');
            installation_path = strrep(installation_path, '\', '/');
        end

        % Configure system libraries based on installation type
        if franka_toolbox_libfranka_system_installation_get()
            addSysLibs(objBuildInfo, {'franka'});
        else
            % Add link flags
            addLinkFlags(objBuildInfo, {['-Wl,-rpath,', strjoin({...
                '$(MATLAB_WORKSPACE)', ...
                strrep(installation_path, ' ', '_'), ...
                'libfranka_arm', 'build', 'usr', 'lib'}, '/')]});
            
            % Add link objects and non-build files
            addLinkObjects(objBuildInfo, ...
                {'libfranka.so'}, ...
                {fullfile(franka_toolbox_installation_path_get(), 'libfranka_arm', 'build', 'usr', 'bin')}, ...
                1000, true, true);
            
            dirPath = fullfile(franka_toolbox_installation_path_get(), 'libfranka_arm', 'build', 'usr', 'lib');
            files = dir(fullfile(dirPath, '*.so*'));
            fileNames = {files.name};
            addNonBuildFiles(objBuildInfo, fileNames, dirPath);
        end

    % Host Linux PC as target configuration
    else
        % Check for Windows compatibility
        if ispc()
            error('Only building with Jetson Hardware Support package is supported in Windows for the Franka Toolbox!')
        end

        % Configure external mode patch
        if strcmp(get_param(objBuildInfo.ComponentName, 'HardwareBoard'), 'None')    
            setTargetProvidesMain(objBuildInfo, true);
            rt_main_src_idx = find(strcmp({objBuildInfo.Src.Files.FileName}, 'rt_main.cpp'));
            if rt_main_src_idx
                objBuildInfo.Src.Files(rt_main_src_idx) = [];
            end
            addSourceFiles(objBuildInfo, 'rt_main.cpp', fullfile(franka_toolbox_installation_path_get(),'franka_toolbox_simulink_library','rtw','src'));
        end
    
        addIncludePaths(objBuildInfo,...
            {fullfile(franka_toolbox_installation_path_get(),'common','include') ...
            fullfile(franka_toolbox_installation_path_get(),'libfranka','include') ...
            fullfile(franka_toolbox_installation_path_get(),'libfranka','common','include')});
        
        if franka_toolbox_libfranka_system_installation_get()
            addLinkFlags(objBuildInfo,{['-Wl,-rpath,"','/opt/openrobots/lib','"']});
            addSysLibPaths(objBuildInfo,{'/opt/openrobots/lib'});
        else
            addLinkFlags(objBuildInfo,...
                {['-Wl,-rpath,"',fullfile(franka_toolbox_installation_path_get(),'libfranka','build','usr','lib'),':/opt/openrobots/lib"']});
            addSysLibPaths(objBuildInfo,{fullfile(franka_toolbox_installation_path_get(),'libfranka','build','usr','bin')});
        end
        
        addSysLibs(objBuildInfo,{'franka'});
        addLinkObjects(objBuildInfo,{'libfranka_matlab'},{fullfile(franka_toolbox_installation_path_get(),'common','bin')},1000,true,true);

    end

end