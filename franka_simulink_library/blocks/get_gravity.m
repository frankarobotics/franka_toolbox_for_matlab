function get_gravity(block)
%  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
%  This file is subject to the terms and conditions defined in the file
%  'LICENSE' , which is part of this package

setup(block);

function setup(block)

    % Register parameters
    block.NumDialogPrms     = 3;
    block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable'};

    robot_state_only = boolean(block.DialogPrm(2).Data);
    gravity_earth = boolean(block.DialogPrm(3).Data);

    if robot_state_only, numInputs = 0; else, numInputs = 3; end
    if gravity_earth, numInputs = numInputs + 1;  end
    
    % Register number of ports
    block.NumInputPorts  = numInputs;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    if ~robot_state_only
        block.InputPort(1).Dimensions  = 7;
        block.InputPort(1).DatatypeID  = 0;  % double
        block.InputPort(1).Complexity  = 'Real';
        block.InputPort(1).DirectFeedthrough = true;

        block.InputPort(2).Dimensions  = 1;
        block.InputPort(2).DatatypeID  = 0;  % double
        block.InputPort(2).Complexity  = 'Real';
        block.InputPort(2).DirectFeedthrough = true;

        block.InputPort(3).Dimensions  = 3;
        block.InputPort(3).DatatypeID  = 0;  % double
        block.InputPort(3).Complexity  = 'Real';
        block.InputPort(3).DirectFeedthrough = true;
    end

    if gravity_earth
        block.InputPort(numInputs).Dimensions  = 3;
        block.InputPort(numInputs).DatatypeID  = 0;  % double
        block.InputPort(numInputs).Complexity  = 'Real';
        block.InputPort(numInputs).DirectFeedthrough = true;
    end

    block.OutputPort(1).Dimensions  = 7;
    block.OutputPort(1).DatatypeID  = 0;  % double
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';
    
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
    block.NumDworks = 1;
    
    block.Dwork(1).Name = 'DWORK1';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

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
    robot_state_only = block.DialogPrm(2).Data;
    gravity_earth = boolean(block.DialogPrm(3).Data);
    
    robot_id = strrep(robot_ip,'.','');
    
    robot_ip = char(['''',robot_ip,'''']);
    robot_id = char(['''',robot_id,'''']);
    
    block.WriteRTWParam('string', 'robot_ip', robot_ip);
    block.WriteRTWParam('string', 'robot_id', robot_id);
    block.WriteRTWParam('matrix', 'robot_state_only', int8(robot_state_only));
    block.WriteRTWParam('matrix', 'gravity_earth', int8(gravity_earth));

%endfunction