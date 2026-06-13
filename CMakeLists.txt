cmake_minimum_required(VERSION 3.16)
if(APPLE)
  project(ScratchArm6DoFSim LANGUAGES C CXX OBJC OBJCXX)
else()
  project(ScratchArm6DoFSim LANGUAGES C CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/third_party/libs")
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/third_party/imgui-1.92.8-docking")
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMGUI_DIR}/misc/cpp
)

if(APPLE)
    find_library(OPENGL_LIBRARY OpenGL REQUIRED)
    target_link_libraries(imgui PUBLIC glfw ${OPENGL_LIBRARY})
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
endif()

add_executable(sim_arm
    src/main.cpp
    src/Algorithm/Kinematics.cpp
)
target_include_directories(sim_arm PRIVATE src)

if(APPLE)
    target_link_libraries(sim_arm PRIVATE mujoco::mujoco glfw imgui ${OPENGL_LIBRARY})
else()
    target_link_libraries(sim_arm PRIVATE mujoco::mujoco glfw imgui OpenGL::GL)
endif()