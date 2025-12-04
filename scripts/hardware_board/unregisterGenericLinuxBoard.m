function unregisterGenericLinuxBoard
    %UNREGISTERGENERICLINUXBOARD Remove the Generic Linux AMD64 board & related objects.
    %
    %   unregisterGenericLinuxBoard()
    %
    %   Removes all target framework objects created by registerGenericLinuxBoard,
    %   including legacy objects from previous versions.
    %
    %   See also: registerGenericLinuxBoard
    
    % List of all object names we've ever used
    names.Target           = "Generic Linux AMD64 Target";
    names.Board            = "Generic Linux AMD64 Board";
    names.TargetConnection = "Generic Linux TCP Connection";
    names.Processor        = "Generic Intel-x86-64 (Linux)";
    names.ExternalMode     = "Generic Linux XCP External Mode";
    names.ExecService      = "SSH Execution Service";
    
    % Intermediate XCP objects that must be cleaned up explicitly if they persist
    names.XCPConnectivity  = "XCP Connectivity";
    names.XCPConfig        = "XCP Configuration";
    names.XCPTransport     = "XCP TCP Transport";
    names.XCPPlatform      = "XCP Platform Abstraction";

    % Legacy names just in case
    legacyNames = [
        "Generic Linux XCP Connection"
        "Generic Linux XCP-over-TCP"
        "Generic Linux TCP/IP Interface"
        "SSH Run on Generic Linux"
    ];
    
    removed = {};
    
    fprintf('Unregistering Generic Linux Board components...\n');
    
    % 1) Remove Target container FIRST
    removeObject('Target', names.Target);
    
    % 2) Remove Connection
    removeObject('TargetConnection', names.TargetConnection);
    
    % 3) Remove Board
    removeObject('Board', names.Board);
    
    % 4) Remove Processor
    removeObject('Processor', names.Processor);
    
    % 4.5) Remove implicitly created TCPChannel
    removeObject('TCPChannel', "Generic Linux TCP Connection TCPChannel");
    
    % 5) Cleanup others
    removeObject('ExecutionService', names.ExecService);
    removeObject('ExternalMode', names.ExternalMode);
    
    % 6) Cleanup XCP sub-objects (important because they can block re-registration)
    removeObject('XCPExternalModeConnectivity', names.XCPConnectivity);
    removeObject('XCP', names.XCPConfig);
    removeObject('XCPTCPIPTransport', names.XCPTransport);
    removeObject('XCPPlatformAbstraction', names.XCPPlatform);

    % 7) Cleanup legacy names
    for i = 1:length(legacyNames)
        cleanLegacy(legacyNames(i));
    end

    fprintf('Unregister complete.\n');
    
    %----------------------------------------------------------------------
    function removeObject(type, name)
        try
            obj = target.get(type, name);
            target.remove(obj); 
            fprintf('  Removed %s: %s\n', type, name);
        catch ME
            % Only report if it's NOT a "not found" error
            if ~strcmp(ME.identifier, 'Target:Object:GetCheck') && ...
               ~contains(ME.message, 'does not name a class')
                 % fprintf('  Error checking %s "%s": %s\n', type, name, ME.message);
            end
        end
    end

    function cleanLegacy(name)
        types = {'Target', 'Board', 'TargetConnection', 'ExternalMode', ...
                 'CommunicationInterface', 'ExecutionService', 'Tool'};
        for k = 1:length(types)
            try
                obj = target.get(types{k}, name);
                target.remove(obj);
                fprintf('  Removed legacy %s: %s\n', types{k}, name);
            catch
            end
        end
    end
end
