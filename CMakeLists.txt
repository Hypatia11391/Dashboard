cmake_minimum_required(VERSION 3.16)
project(Dashboard LANGUAGES CXX)

# Require C++20 for WPILib
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define WPILib installation root and platform
set(WPILIB_YEAR "2026")
if(WIN32)
    set(WPILIB_ROOT "C:/Users/Public/wpilib/${WPILIB_YEAR}")
    set(WPILIB_PLATFORM "windows/x86-64")
else()
    set(WPILIB_ROOT "$ENV{HOME}/wpilib/${WPILIB_YEAR}")
    set(WPILIB_PLATFORM "linux/x86-64")
endif()

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# --- ImGui Target ---
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui)

add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

target_link_libraries(imgui PUBLIC glfw OpenGL::GL ${CMAKE_DL_LIBS})

# --- Main Dashboard Executable ---
add_executable(Dashboard 
    main.cpp 
    src/dashboard.cpp
    src/primitives.cpp
)

set(WPILIB_LIB_DIR "${WPILIB_ROOT}/extracted/${WPILIB_PLATFORM}/shared")

target_include_directories(Dashboard PRIVATE
    ${WPILIB_ROOT}/extracted/include
    ${CMAKE_SOURCE_DIR}
)

target_link_directories(Dashboard PRIVATE ${WPILIB_LIB_DIR})

# Allow Linux executable to locate WPILib .so files automatically
set_target_properties(Dashboard PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${WPILIB_LIB_DIR}"
)

target_link_libraries(Dashboard PRIVATE
    imgui
    wpilibc
    ntcore
    cameraserver
    cscore
    wpiHal
    wpimath
    wpinet
    wpiutil
)