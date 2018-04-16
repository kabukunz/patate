########################################################################
# Vitelotte Editor                                                     #
########################################################################

find_package(OpenGL)
find_package(GLEW)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)

set(mvg_editor_MISSING "")
if(NOT OPENGL_FOUND)
    set(mvg_editor_MISSING "${mvg_editor_MISSING} OpenGL")
endif()
if(NOT GLEW_FOUND)
    set(mvg_editor_MISSING "${mvg_editor_MISSING} GLEW")
endif()
if(NOT Qt5Widgets_FOUND)
    set(mvg_editor_MISSING "${mvg_editor_MISSING} Qt5Widgets")
endif()
if(NOT Qt5OpenGL_FOUND)
    set(mvg_editor_MISSING "${mvg_editor_MISSING} Qt5OpenGL")
endif()

if(NOT mvg_editor_MISSING)
    set(CMAKE_AUTOMOC ON)

    if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER "3.9")
        cmake_policy(SET CMP0071 NEW)
    endif()

    # Remove a warning triggred by Qt.
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "-Wno-long-long ${CMAKE_CXX_FLAGS}")
    endif(CMAKE_COMPILER_IS_GNUCXX)

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
    add_dependencies(vitelotte_examples mvg_editor)
    add_dependencies(mvg_editor vitelottes_vg_mesh_renderer_shaders)

    target_link_libraries(mvg_editor
        vitelotte_examples_common
        Qt5::Widgets
        Qt5::OpenGL
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
    )
else()
    message(WARNING "Skipping mvg_editor because of the following missing dependencies:${mvg_editor_MISSING}")
endif()

