cmake_minimum_required(VERSION 3.5)

project(franka_matlab
  VERSION 1.0.0
  LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

find_package(Eigen3 REQUIRED)

# Set default libfranka path
set(FRANKA_FOLDER "libfranka" CACHE STRING "Path to libfranka directory")

# Define Franka paths
set(Franka_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/include)
set(Franka_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/build/libfranka${CMAKE_SHARED_LIBRARY_SUFFIX})

file(GLOB common_SRC
     "src/*.cpp"
)

## Library
add_library(franka_matlab STATIC
  ${common_SRC}
)

set_target_properties(franka_matlab PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  EXPORT_NAME FrankaMatlab
)

target_compile_features(franka_matlab INTERFACE
  cxx_attribute_deprecated
  cxx_constexpr
  cxx_defaulted_functions
  cxx_deleted_functions
  cxx_generalized_initializers
  cxx_noexcept
  cxx_uniform_initialization
)

target_include_directories(franka_matlab PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/common/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(franka_matlab PRIVATE
  Eigen3::Eigen3
  ${Franka_LIBRARIES}
)