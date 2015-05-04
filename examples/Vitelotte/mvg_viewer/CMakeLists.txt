###############################################################################
# Vitelotte GL Viewer                                                         #
###############################################################################

project(MvgViewer)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(X11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${GLFW3_INCLUDE_DIR}
    "../../.."
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(mvg_viewer_SRCS
    ../common/trackball.cpp
    ../common/shaders.cpp
    ../common/glPointRenderer.cpp
    ../common/glLineRenderer.cpp
    ../common/vgNodeRenderer.cpp
    mvgViewer.cpp
)
add_executable(mvg_viewer ${mvg_viewer_SRCS})
add_dependencies(examples mvg_viewer)

target_link_libraries(mvg_viewer
    ${GLFW3_LIBRARIES}
    ${GLEW_LIBRARIES}
)
