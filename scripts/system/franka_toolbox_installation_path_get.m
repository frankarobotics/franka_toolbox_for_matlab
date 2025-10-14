function installation_path = franka_toolbox_installation_path_get()
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    installation_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

end