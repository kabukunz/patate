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

set(mvg_editor_SHADERS
	shaders/vert_points.glsl
	shaders/geom_points.glsl
	shaders/frag_points.glsl
	shaders/vert_lines.glsl
	shaders/geom_lines.glsl
	shaders/frag_lines.glsl
)

qt5_add_resources(mvg_editor_RES
	editor.qrc
)

set(mvg_editor_SRCS
    ../common/orthographicCamera.cpp
    ../common/glPointRenderer.cpp
    ../common/glLineRenderer.cpp
    ../common/vgMeshWithCurves.cpp
    ../common/vgMeshWithCurvesReader.cpp
    ../common/vgNodeRenderer.cpp
    ../common/shaders.cpp
    document.cpp
    valueEditor.cpp
    editor.cpp
    mainWindow.cpp
    ${mvg_editor_RES}
    main.cpp
)

add_custom_command(
	OUTPUT ${PROJECT_SOURCE_DIR}/shaders.cpp
    COMMAND python3 ${PROJECT_SOURCE_DIR}/../../../bin/shader2cpp -o shaders.cpp -e ${mvg_editor_SHADERS}
    DEPENDS ${mvg_editor_SHADERS}
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	VERBATIM
)
#set_source_files_properties(glLineRenderer.cpp
#	OBJECT_DEPENDS ${PROJECT_SOURCE_DIR}/shaders.hpp)

add_executable(mvg_editor
    ${mvg_editor_SRCS}
)
add_dependencies(examples mvg_editor)

target_link_libraries(mvg_editor
    Qt5::Widgets
    Qt5::OpenGL
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
)
