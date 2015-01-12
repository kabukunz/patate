###############################################################################
# Vitelotte GL2 Viewer                                                        #
###############################################################################

project(VitelotteGL2Viewer)

cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ../../..
)

if(EMSCRIPTEN)
    add_definitions(-DEMSCRIPTEN -D VITELOTTE_USE_OPENGL_ES -Wno-warn-absolute-paths -std=c++11 --bind)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --embed-file ${PROJECT_BINARY_DIR}/files@/ --bind -s TOTAL_MEMORY=134217728 -s NO_EXIT_RUNTIME=1")
else()
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(SDL REQUIRED)

    include_directories(
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${SDL_INCLUDE_DIR}
    )

    set(gl2Viewer_LIBRARIES
        ${SDL_LIBRARY}
        ${GLEW_LIBRARIES}
        ${OPENGL_LIBRARIES}
    )
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(gl2Viewer_SRCS
    ../common/orthographicCamera.cpp
    ../common/shaders.cpp
    ../common/glPointRenderer.cpp
    ../common/glLineRenderer.cpp
    ../common/vgNodeRenderer.cpp
    gl2Viewer.cpp
)
add_executable(gl2_viewer ${gl2Viewer_SRCS})

target_link_libraries(gl2_viewer
#    "/usr/lib/x86_64-linux-gnu/libSDL2.so"
    ${gl2Viewer_LIBRARIES}
)
