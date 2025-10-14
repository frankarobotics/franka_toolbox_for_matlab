function apply_control(block)
%  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
%  This file is subject to the terms and conditions defined in the file
%  'LICENSE' , which is part of this package

setup(block);

function setup(block)

    % Register parameters
    block.NumDialogPrms     = 15;
    block.DialogPrmsTunable = {'Nontunable', ...    % robot_ip
                               'Nontunable', ...    % gripper_attached
                               'Nontunable', ...    % control_mode
                               'Nontunable', ...    % controller_choice
                               'Tunable', ...       % collision_thresholds
                               'Tunable', ...       % joint_impedance
                               'Tunable', ...       % cartesian_impedance
                               'Tunable', ...       % load_inertia
                               'Tunable', ...       % NE_T_EE
                               'Tunable', ...       % EE_T_K
                               'Nontunable', ...    % init_joint_configuration_req
                               'Tunable', ...       % init_joint_configuration
                               'Nontunable',...     % elbow_control
                               'Nontunable',...     % rate_limiter
                               'Tunable'};          % cuttoff_frequency
    
    gripper_selection = block.DialogPrm(2).Data;
    control_mode = block.DialogPrm(3).Data;
    elbow_control = block.DialogPrm(13).Data;

    switch gripper_selection

        case 1

            gripper_attached = 0;
            vacuum_gripper_attached = 0;

        case 2

            gripper_attached = 1;
            vacuum_gripper_attached = 0;

        case 3

            gripper_attached = 0;
            vacuum_gripper_attached = 1;

    end
    
    if (boolean(elbow_control))
        
        switch control_mode
            
            case FrankaRobotControlModes.torqueControlCartesianPose.real
                
               control_mode = FrankaRobotControlModes.torqueControlCartesianPoseWithElbow.real;
              
            case FrankaRobotControlModes.torqueControlCartesianVelocities.real
                
               control_mode = FrankaRobotControlModes.torqueControlCartesianVelocitiesWithElbow.real;
               
            case FrankaRobotControlModes.cartesianPoseControllerMode.real
                
                control_mode = FrankaRobotControlModes.cartesianPoseControllerModeWithElbow.real;
                
            case FrankaRobotControlModes.cartesianVelocitiesControllerMode.real
                
                control_mode = FrankaRobotControlModes.cartesianVelocitiesControllerModeWithElbow.real;
                
        end
        
    end
    
    controlModeSizes = struct('numInputs',{},'width',{});
    
    controlModeSizes(1).numInputs = 1;
    controlModeSizes(1).width = [7, 0, 0];
    
    controlModeSizes(2).numInputs = 2;
    controlModeSizes(2).width = [7, 7, 0];
    
    controlModeSizes(3).numInputs = 2;
    controlModeSizes(3).width = [7, 7, 0];
    
    controlModeSizes(4).numInputs = 2;
    controlModeSizes(4).width = [16, 7, 0];
    
    controlModeSizes(5).numInputs = 2;
    controlModeSizes(5).width = [6, 7, 0];
    
    controlModeSizes(6).numInputs = 1;
    controlModeSizes(6).width = [7, 0, 0];
    
    controlModeSizes(7).numInputs = 1;
    controlModeSizes(7).width = [7, 0, 0];
    
    controlModeSizes(8).numInputs = 1;
    controlModeSizes(8).width = [16, 0, 0];
    
    controlModeSizes(9).numInputs = 1;
    controlModeSizes(9).width = [6, 0, 0];
    
    controlModeSizes(10).numInputs = 3;
    controlModeSizes(10).width = [16, 2, 7];
    
    controlModeSizes(11).numInputs = 3;
    controlModeSizes(11).width = [6, 2, 7];
    
    controlModeSizes(12).numInputs = 2;
    controlModeSizes(12).width = [16, 2, 0];
    
    controlModeSizes(13).numInputs = 2;
    controlModeSizes(13).width = [6, 2, 0];
    
    % Register number of ports
    block.NumInputPorts  = controlModeSizes(control_mode).numInputs + gripper_attached + vacuum_gripper_attached;
    block.NumOutputPorts = 0;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    
    % Override input port properties
    if (boolean(gripper_attached))
        block.InputPort(1).Dimensions  = [9 1];
        block.InputPort(1).DatatypeID  = 0;  % double
        block.InputPort(1).Complexity  = 'Real';
        block.InputPort(1).DirectFeedthrough = true;
    end

    if (boolean(vacuum_gripper_attached))
        block.InputPort(1).Dimensions  = [6 1];
        block.InputPort(1).DatatypeID  = 0;  % double
        block.InputPort(1).Complexity  = 'Real';
        block.InputPort(1).DirectFeedthrough = true;
    end
    
    for ii=gripper_attached+vacuum_gripper_attached+1:controlModeSizes(control_mode).numInputs + gripper_attached + vacuum_gripper_attached
        % Override input port properties
        block.InputPort(ii).Dimensions  = [controlModeSizes(control_mode).width(ii-gripper_attached-vacuum_gripper_attached) 1];
        block.InputPort(ii).DatatypeID  = 0;  % double
        block.InputPort(ii).Complexity  = 'Real';
        block.InputPort(ii).DirectFeedthrough = true;
        
    end


    % Register sample times
    %  [0 offset]            : Continuous sample time
    %  [positive_num offset] : Discrete sample time
    %
    %  [-1, 0]               : Inherited sample time
    %  [-2, 0]               : Variable sample time
    block.SampleTimes = [.001 0];

    % Specify the block simStateCompliance. The allowed values are:
    %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
    %    'DefaultSimState', < Same sim state as a built-in block
    %    'HasNoSimState',   < No sim state
    %    'CustomSimState',  < Has GetSimState and SetSimState methods
    %    'DisallowSimState' < Error out when saving or restoring the model sim state
    block.SimStateCompliance = 'DefaultSimState';

    %   %% Register methods
    block.RegBlockMethod('CheckParameters',         @CheckPrms);
    block.RegBlockMethod('ProcessParameters',       @ProcessPrms);
    block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
    block.RegBlockMethod('Start',                   @Start); 
    block.RegBlockMethod('WriteRTW',                @WriteRTW);
    block.RegBlockMethod('Outputs',                 @Outputs);

    % %% Block runs on TLC in accelerator mode.
    %     block.SetAccelRunOnTLC(true);

function CheckPrms(block)
%endfunction

function DoPostPropSetup(block)
    % Setup Dwork
    block.NumDworks = 2;
    
    block.Dwork(1).Name = 'DWORK1';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
    
    block.Dwork(2).Name = 'DWORK2';
    block.Dwork(2).Dimensions      = 1;
    block.Dwork(2).DatatypeID      = 0;
    block.Dwork(2).Complexity      = 'Real';
    block.Dwork(2).UsedAsDiscState = true;
    
    % Register all tunable parameters as runtime parameters.
    block.AutoRegRuntimePrms;

%endfunction

function ProcessPrms(block)
    block.AutoUpdateRuntimePrms;
 
%endfunction

function Start(block)
%endfunction

function Outputs(block)
%endfunction

function WriteRTW(block)

    robot_ip = block.DialogPrm(1).Data;
    gripper_selection = block.DialogPrm(2).Data;
    control_mode = block.DialogPrm(3).Data;
    controller_choice = block.DialogPrm(4).Data;
    collision_thresholds = block.DialogPrm(5).Data;
    joint_impedance = block.DialogPrm(6).Data;
    cartesian_impedance = block.DialogPrm(7).Data;
    load_inertia = block.DialogPrm(8).Data;
    NE_T_EE = block.DialogPrm(9).Data;
    EE_T_K = block.DialogPrm(10).Data;
    init_joint_configuration_req = block.DialogPrm(11).Data;
    init_joint_configuration = block.DialogPrm(12).Data;
    elbow_control = block.DialogPrm(13).Data;
    rate_limiter = block.DialogPrm(14).Data;
    cutoff_frequency = block.DialogPrm(15).Data;

    switch gripper_selection

        case 1

            gripper_attached = 0;
            vacuum_gripper_attached = 0;

        case 2

            gripper_attached = 1;
            vacuum_gripper_attached = 0;

        case 3

            gripper_attached = 0;
            vacuum_gripper_attached = 1;

    end
    
    robot_id = strrep(robot_ip,'.','');
    
    if (boolean(elbow_control))
        
        switch control_mode
            
            case FrankaRobotControlModes.torqueControlCartesianPose.real
                
               control_mode = FrankaRobotControlModes.torqueControlCartesianPoseWithElbow.real;
              
            case FrankaRobotControlModes.torqueControlCartesianVelocities.real
                
               control_mode = FrankaRobotControlModes.torqueControlCartesianVelocitiesWithElbow.real;
               
            case FrankaRobotControlModes.cartesianPoseControllerMode.real
                
                control_mode = FrankaRobotControlModes.cartesianPoseControllerModeWithElbow.real;
                
            case FrankaRobotControlModes.cartesianVelocitiesControllerMode.real
                
                control_mode = FrankaRobotControlModes.cartesianVelocitiesControllerModeWithElbow.real;
                
        end
        
    end
    
    robot_ip = char(['''',robot_ip,'''']);
    robot_id = char(['''',robot_id,'''']);
    
    block.WriteRTWParam('string', 'robot_ip', robot_ip);
    block.WriteRTWParam('string', 'robot_id', robot_id);
    block.WriteRTWParam('matrix', 'gripper_attached', int8(gripper_attached));
    block.WriteRTWParam('matrix', 'vacuum_gripper_attached', int8(vacuum_gripper_attached));
    block.WriteRTWParam('matrix', 'control_mode', int8(control_mode));
    block.WriteRTWParam('matrix', 'controller_choice', int8(controller_choice));
    block.WriteRTWParam('matrix', 'collision_thresholds', collision_thresholds);
    block.WriteRTWParam('matrix', 'joint_impedance', joint_impedance);
    block.WriteRTWParam('matrix', 'cartesian_impedance', cartesian_impedance);
    block.WriteRTWParam('matrix', 'load_inertia', load_inertia);
    block.WriteRTWParam('matrix', 'NE_T_EE', NE_T_EE);
    block.WriteRTWParam('matrix', 'EE_T_K', EE_T_K);
    block.WriteRTWParam('matrix', 'init_joint_configuration_req', int8(init_joint_configuration_req));
    block.WriteRTWParam('matrix', 'init_joint_configuration', init_joint_configuration);
    block.WriteRTWParam('matrix', 'elbow_control', int8(elbow_control));
    block.WriteRTWParam('matrix', 'rate_limiter', int8(rate_limiter));
    block.WriteRTWParam('matrix', 'cutoff_frequency', cutoff_frequency);

%endfunction