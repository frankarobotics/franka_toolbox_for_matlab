function registerGenericLinuxBoard(varargin)
    %REGISTERGENERICLINUXBOARD One-off registration of a generic Linux AMD64 board
    %   registerGenericLinuxBoard()
    %   registerGenericLinuxBoard('force')
    %
    % This function registers a custom hardware board for deploying Simulink
    % models to remote Linux x86-64 targets via SSH. It provides:
    %
    %   - A "Generic Linux AMD64 Board" hardware target
    %   - XCP-over-TCP external mode connectivity
    %   - SSH-based execution tool
    %   - Configurable target connection (IP/port)
    %
    % All objects are registered with target.add(..., 'UserInstall', true)
    % so they persist across MATLAB sessions.
    %
    % See also: unregisterGenericLinuxBoard
    
    force = (nargin > 0) && strcmpi(varargin{1}, 'force');
    
    boardName = "Generic Linux AMD64 Board";
    connName  = "Generic Linux TCP Connection";
    procName  = "Generic Intel-x86-64 (Linux)";
    
    %----------------------------------------------------------------------
    % 0) Check if fully registered; if not, clean up and re-register
    %----------------------------------------------------------------------
    [isReg, details] = isFullyRegistered(boardName, connName);
    
    if ~force && isReg
        fprintf('Generic Linux AMD64 Board is already registered.\n');
        fprintf('Found existing objects:\n%s\n', details);
        fprintf('Use registerGenericLinuxBoard(''force'') to overwrite.\n');
        return;
    end
    
    % Clean slate first
    if force
        fprintf('Forcing registration: removing existing objects...\n');
    end
    
    % Always try to unregister first if we are here (either forcing, or partial state)
    try
        unregisterGenericLinuxBoard();
    catch
    end
    
    fprintf('Registering new board objects...\n');
    
    %----------------------------------------------------------------------
    % 1) Processor: reuse stock or create custom
    %----------------------------------------------------------------------
    proc = [];
    procCreated = false;
    try
        proc = target.get('Processor', "Intel-x86-64 (Linux 64)");
        fprintf('  Using stock Processor: Intel-x86-64 (Linux 64)\n');
    catch
        try
            proc = target.get('Processor', procName);
            fprintf('  Found existing custom Processor: %s\n', procName);
        catch
            proc = target.create('Processor', ...
                'Name', procName, ...
                'Manufacturer', "Intel", ...
                'ClockRate', 1000);
            procCreated = true;
            fprintf('  Created new Processor: %s\n', procName);
        end
    end
    
    %----------------------------------------------------------------------
    % 2) Create Board
    %----------------------------------------------------------------------
    board = target.create('Board', 'Name', boardName);
    board.Processors = proc;
    fprintf('  Created Board: %s\n', boardName);
    
    %----------------------------------------------------------------------
    % 3) Communication Interface (TCP/IP rtiostream)
    %----------------------------------------------------------------------
    rtiostreamSrc = fullfile(matlabroot, 'toolbox', 'coder', 'rtiostream', ...
        'src', 'rtiostreamtcpip', 'rtiostream_tcpip.c');
    
    buildDeps = target.create('BuildDependencies', ...
        'SourceFiles', {rtiostreamSrc});
    
    apiImpl = target.create('APIImplementation', ...
        'Name', 'TCP RTIOStream Implementation', ...
        'API', target.get('API', 'rtiostream'), ...
        'BuildDependencies', buildDeps);
    
    comIf = target.create('CommunicationInterface', ...
        'Name', 'Generic Linux TCP Interface', ...
        'Channel', 'TCPChannel', ...
        'APIImplementations', apiImpl);
    
    board.CommunicationInterfaces = comIf;
    
    %----------------------------------------------------------------------
    % 4) External Mode (XCP over TCP)
    %----------------------------------------------------------------------
    % Use a simple setup that relies on defaults for transport/platform
    xcpTransport = target.create('XCPTCPIPTransport', ...
        'Name', 'XCP TCP Transport');
    
    % Set properties after creation since they are not constructor arguments
    % Note: target.XCPTCPIPTransport properties might differ by MATLAB version.
    % If IPAddress is not a property, we rely on the TargetConnection object (conn)
    % to handle the address, and XCP will bind to all interfaces (0.0.0.0).
    % xcpTransport.IPAddress = '192.168.1.100'; 
    % xcpTransport.Port = 17725;
    % xcpTransport.Verbose = 1;

    % Note: Omitting XCPPlatformAbstraction to use default implementation.
    % If customization is needed later, we must provide a valid header/source.
    xcpConfig = target.create('XCP', ...
        'Name', 'XCP Configuration', ...
        'XCPTransport', xcpTransport);
    
    xcpConnectivity = target.create('XCPExternalModeConnectivity', ...
        'Name', 'XCP Connectivity', ...
        'XCP', xcpConfig);
    
    extMode = target.create('ExternalMode', ...
        'Name', 'Generic Linux XCP External Mode', ...
        'Connectivities', xcpConnectivity);
    
    board.CommunicationProtocolStacks = extMode;
    
    %----------------------------------------------------------------------
    % 5) Execution Tool (SSH via target.ExecutionTool)
    %----------------------------------------------------------------------
    % Using target.ExecutionTool provides getApplicationStatus() which enables
    % full Monitor & Tune, Deploy, Connect, and Start functionality.
    
    % Get the path to our SSHExecutionTool class
    thisDir = fileparts(mfilename('fullpath'));
    
    % Create execution service with our custom implementation
    execSvc = target.create('ExecutionService', ...
        'Name', 'SSH Execution Service');
    
    % Set up the API implementation to use our SSHExecutionTool class
    execSvc.APIImplementation = target.create('APIImplementation', ...
        'Name', 'SSHExecutionToolImplementation');
    
    execSvc.APIImplementation.BuildDependencies = target.create('MATLABDependencies');
    execSvc.APIImplementation.BuildDependencies.Classes = {'SSHExecutionTool'};
    % Ensure the folder containing the class is on the MATLAB path
    addpath(thisDir);
    execSvc.APIImplementation.API = target.get('API', 'ExecutionTool');
    
    % Associate execution service with board
    board.Tools.ExecutionTools = execSvc;
    
    %----------------------------------------------------------------------
    % 6) Target Connection
    %----------------------------------------------------------------------
    conn = target.create('TargetConnection', ...
        'Name', connName, ...
        'Target', board, ...
        'CommunicationType', 'TCPChannel', ...
        'IPAddress', '192.168.122.100', ...
        'Port', '17725');
    fprintf('  Created TargetConnection: %s\n', connName);
    
    %----------------------------------------------------------------------
    % 7) Register objects
    %----------------------------------------------------------------------
    if procCreated
        target.add(proc, 'UserInstall', true, 'SuppressOutput', true);
        fprintf('  Registered Processor.\n');
    end
    target.add(board, 'UserInstall', true, 'SuppressOutput', true);
    fprintf('  Registered Board.\n');
    
    target.add(conn, 'UserInstall', true, 'SuppressOutput', true);
    fprintf('  Registered TargetConnection.\n');
    
    fprintf('Generic Linux AMD64 Board registered successfully.\n');
    fprintf('Select board in Hardware tab, then configure connection IP/port.\n');
end

%--------------------------------------------------------------------------
function [ok, details] = isFullyRegistered(boardName, connName)
    ok = false;
    details = '';
    try
        b = target.get('Board', boardName);
        details = sprintf('%s  - Board: Found (%s)\n', details, string(b.Name));
        
        c = target.get('TargetConnection', connName);
        details = sprintf('%s  - Connection: Found (%s)\n', details, string(c.Name));
        
        ok = true;
    catch
        % Missing something
    end
end
