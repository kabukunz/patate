########################################################################
# Vitelotte Editor                                                     #
########################################################################

project(MvgEditor)

cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    "../../.."
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(mvg_editor_RES
	editor.qrc
)

set(mvg_editor_SRCS
    document.cpp
    valueEditor.cpp
    editor.cpp
    mainWindow.cpp
    ${mvg_editor_RES}
    main.cpp
)

add_executable(mvg_editor
    ${mvg_editor_SRCS}
)
add_dependencies(examples mvg_editor)
add_dependencies(mvg_editor vitelottes_vg_mesh_renderer_shaders)

target_link_libraries(mvg_editor
    vitelotte_examples_common
    Qt5::Widgets
    Qt5::OpenGL
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
)
