cmake_minimum_required(VERSION 3.16)
project(Dashboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(WPILIB_ROOT "$ENV{HOME}/wpilib/2026")
set(WPILIB_LIB_DIR "${WPILIB_ROOT}/extracted/linux/x86-64/shared")
set(WPILIB_INC_DIR "${WPILIB_ROOT}/extracted/include")

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui)
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/implot)
file(GLOB IMGUI_SOURCES 
    "${IMGUI_DIR}/*.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)

add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends ${IMPLOT_DIR})
target_link_libraries(imgui PUBLIC glfw OpenGL::GL ${CMAKE_DL_LIBS})

add_executable(Dashboard 
    main.cpp 
    src/dashboard.cpp
    src/primitives.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

target_include_directories(Dashboard PRIVATE ${WPILIB_INC_DIR} ${CMAKE_SOURCE_DIR})
target_link_directories(Dashboard PRIVATE ${WPILIB_LIB_DIR})

set_target_properties(Dashboard PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${WPILIB_LIB_DIR}"
)

target_link_libraries(Dashboard PRIVATE
    imgui
    ntcore
    wpinet
    wpiutil
)