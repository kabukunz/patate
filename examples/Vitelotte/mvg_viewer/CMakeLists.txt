###############################################################################
# Vitelotte GL Viewer                                                         #
###############################################################################

project(MvgViewer)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(X11)
find_package(OpenGL)
find_package(GLEW)
find_package(GLFW3)

set(mvg_viewer_MISSING "")
if(NOT X11_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} X11")
endif()
if(NOT OPENGL_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} OpenGL")
endif()
if(NOT GLEW_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} GLEW")
endif()
if(NOT GLFW3_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} GLFW3")
endif()

if(NOT mvg_viewer_MISSING)
    include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${GLFW3_INCLUDE_DIR}
        "../../.."
    )

    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    set(mvg_viewer_SRCS
        mvgViewer.cpp
    )
    add_executable(mvg_viewer ${mvg_viewer_SRCS})
    add_dependencies(vitelotte_examples mvg_viewer)
    add_dependencies(mvg_viewer vitelottes_vg_mesh_renderer_shaders)

    target_link_libraries(mvg_viewer
        vitelotte_examples_common
        ${GLFW3_LIBRARIES}
        ${GLEW_LIBRARIES}
    )
else()
    message(WARNING "Skipping mvg_viewer because of the following missing dependencies:${mvg_viewer_MISSING}")
endif()

