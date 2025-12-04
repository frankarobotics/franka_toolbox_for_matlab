cmake_minimum_required(VERSION 3.22)
project(franka_toolbox_simulink)
find_package(Matlab REQUIRED)

# Manually specify source files
set(franka_toolbox_simulink_SRC
    "blocks/get_duration_period.cpp"
    "blocks/get_robot_state.cpp"
    "blocks/get_gripper_state.cpp"
    "blocks/get_vacuum_gripper_state.cpp"
)

set(Matlab_INCLUDE_DIRS 
     ${Matlab_INCLUDE_DIRS}
     ${Matlab_ROOT_DIR}/simulink/include
)

# Create a library for local_utils
add_library(local_utils OBJECT "blocks/local_utils.cpp")
target_include_directories(local_utils PRIVATE ${Matlab_INCLUDE_DIRS})

foreach(franka_toolbox_simulink_function ${franka_toolbox_simulink_SRC})
     get_filename_component(sfunction ${franka_toolbox_simulink_function} NAME_WLE [CACHE])
     matlab_add_mex(NAME ${sfunction} 
          SRC ${franka_toolbox_simulink_function}
          $<TARGET_OBJECTS:local_utils>
     )
endforeach()
