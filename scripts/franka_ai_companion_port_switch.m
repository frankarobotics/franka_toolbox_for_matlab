function franka_ai_companion_port_switch(port)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    port_d = port;

    if isstring(port)
        port_d = str2double(port);
    end

    if ischar(port)
        port_d = str2double(port);
    end

    hd = nvidiacoder.internal.BoardParameters('NVIDIA Jetson');
    setParam(hd,'port',port_d);

end