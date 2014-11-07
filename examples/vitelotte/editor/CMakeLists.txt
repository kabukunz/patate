########################################################################
# Vitelotte Editor                                                     #
########################################################################

project(VitelotteEditor)

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

set(vitelotte_editor_SRCS
    document.cpp
    orthographicCamera.cpp
    valueEditor.cpp
    editor.cpp
    main.cpp
)

add_executable(editor
	${vitelotte_editor_SRCS}
)

target_link_libraries(editor
    Qt5::Widgets
    Qt5::OpenGL
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
)
