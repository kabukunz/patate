###############################################################################
# Vitelotte GL Viewer                                                         #
###############################################################################

project(VitelotteGLViewer)

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

set(vitelotte_SRCS
    ../common/trackball.cpp
    glViewer.cpp
)
add_executable(gl_viewer ${vitelotte_SRCS})

target_link_libraries(gl_viewer
    ${GLFW3_LIBRARIES}
    ${GLEW_LIBRARIES}
)
