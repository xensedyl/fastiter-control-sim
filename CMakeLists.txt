cmake_minimum_required(VERSION 3.16)

project(fr3_control_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(PYBIND11_FINDPYTHON ON)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)

# Prefer the active mamba/conda prefix.  Without this explicit hint, pkg-config
# may silently pick a system Pinocchio (often 3.4) even after `mamba activate`.
if(DEFINED ENV{CONDA_PREFIX})
  set(FR3_SIM_CONDA_PREFIX "$ENV{CONDA_PREFIX}")
else()
  set(FR3_SIM_CONDA_PREFIX "")
endif()
set(FR3_SIM_PKG_CONFIG_PATHS)
if(FR3_SIM_CONDA_PREFIX)
  list(APPEND FR3_SIM_PKG_CONFIG_PATHS
    "${FR3_SIM_CONDA_PREFIX}/lib/pkgconfig"
    "${FR3_SIM_CONDA_PREFIX}/share/pkgconfig"
  )
endif()
if(FR3_SIM_PKG_CONFIG_PATHS)
  string(JOIN ":" FR3_SIM_PKG_CONFIG_PREFIX ${FR3_SIM_PKG_CONFIG_PATHS})
  if(DEFINED ENV{PKG_CONFIG_PATH} AND NOT "$ENV{PKG_CONFIG_PATH}" STREQUAL "")
    set(FR3_SIM_PKG_CONFIG_PREFIX
      "${FR3_SIM_PKG_CONFIG_PREFIX}:$ENV{PKG_CONFIG_PATH}")
  endif()
  set(ENV{PKG_CONFIG_PATH} "${FR3_SIM_PKG_CONFIG_PREFIX}")
endif()
pkg_check_modules(PINOCCHIO REQUIRED IMPORTED_TARGET pinocchio>=3.2)
if(FR3_SIM_CONDA_PREFIX)
  find_library(TINYXML2_LIBRARY NAMES tinyxml2
    PATHS "${FR3_SIM_CONDA_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
  find_library(CONSOLE_BRIDGE_LIBRARY NAMES console_bridge
    PATHS "${FR3_SIM_CONDA_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
else()
  find_library(TINYXML2_LIBRARY NAMES tinyxml2 REQUIRED)
  find_library(CONSOLE_BRIDGE_LIBRARY NAMES console_bridge REQUIRED)
endif()

add_library(fr3_kinematics STATIC
  cpp/src/robot_model.cpp
)
target_include_directories(fr3_kinematics
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
)
target_link_libraries(fr3_kinematics
  PUBLIC
    Eigen3::Eigen
    PkgConfig::PINOCCHIO
    ${TINYXML2_LIBRARY}
    ${CONSOLE_BRIDGE_LIBRARY}
)
target_compile_options(fr3_kinematics PRIVATE -Wall -Wextra -Wpedantic)

pybind11_add_module(_fr3_sim MODULE
  cpp/src/bindings.cpp
)
target_link_libraries(_fr3_sim PRIVATE fr3_kinematics)
target_compile_options(_fr3_sim PRIVATE -Wall -Wextra -Wpedantic)

set(FR3_SIM_PYTHON_PACKAGE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/python/fr3_control_sim"
  CACHE PATH "Output directory for the Python extension module"
)
set(FR3_SIM_RUNTIME_PATHS "/usr/local/lib")
if(FR3_SIM_CONDA_PREFIX)
  list(PREPEND FR3_SIM_RUNTIME_PATHS "${FR3_SIM_CONDA_PREFIX}/lib")
endif()
set_target_properties(_fr3_sim PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${FR3_SIM_PYTHON_PACKAGE_DIR}"
  BUILD_RPATH "${FR3_SIM_RUNTIME_PATHS}"
  INSTALL_RPATH "${FR3_SIM_RUNTIME_PATHS}"
)

option(FR3_SIM_BUILD_TESTS "Build the C++ smoke test" ON)
if(FR3_SIM_BUILD_TESTS)
  enable_testing()
  add_executable(fr3_kinematics_test cpp/tests/test_kinematics.cpp)
  target_link_libraries(fr3_kinematics_test PRIVATE fr3_kinematics)
  target_compile_definitions(fr3_kinematics_test PRIVATE
    FR3_SIM_DEFAULT_URDF="${CMAKE_CURRENT_SOURCE_DIR}/models/fr3_franka_hand.urdf"
  )
  add_test(NAME fr3_kinematics_smoke COMMAND fr3_kinematics_test)
endif()
