cmake_minimum_required(VERSION 3.16)
if(APPLE)
  project(ScratchArm6DoFSim LANGUAGES C CXX OBJC OBJCXX)
else()
  project(ScratchArm6DoFSim LANGUAGES C CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find pre-built MuJoCo from third_party/libs
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/third_party/libs")
find_package(mujoco REQUIRED)

# GLFW (from third_party/libs)
find_package(glfw3 REQUIRED)

add_executable(sim_arm src/main.cpp)
target_link_libraries(sim_arm PRIVATE mujoco::mujoco glfw)
