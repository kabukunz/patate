###############################################################################
# Vitelotte GL Viewer                                                         #
###############################################################################

project(MvgViewer)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(X11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW3)

if ( NOT GLFW3_FOUND )
    message(WARNING "GLFW3 not found, skipping MvgViewer")
else ( NOT GLFW3_FOUND )
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
    add_dependencies(examples mvg_viewer)
    add_dependencies(mvg_viewer vitelottes_vg_mesh_renderer_shaders)

    target_link_libraries(mvg_viewer
        vitelotte_examples_common
        ${GLFW3_LIBRARIES}
        ${GLEW_LIBRARIES}
    )
endif( NOT GLFW3_FOUND )


