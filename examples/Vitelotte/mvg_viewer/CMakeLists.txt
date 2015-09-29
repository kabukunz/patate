###############################################################################
# Vitelotte GL Viewer                                                         #
###############################################################################


find_package(OpenGL)
find_package(GLEW)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)

set(mvg_viewer_MISSING "")
if(NOT OPENGL_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} OpenGL")
endif()
if(NOT GLEW_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} GLEW")
endif()
if(NOT Qt5Widgets_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} Qt5Widgets")
endif()
if(NOT Qt5OpenGL_FOUND)
    set(mvg_viewer_MISSING "${mvg_viewer_MISSING} Qt5OpenGL")
endif()

if(NOT mvg_viewer_MISSING)
    set(CMAKE_AUTOMOC ON)

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

    set(mvg_viewer_SRCS
        main.cpp
        window.cpp
        view.cpp
    )
    add_executable(mvg_viewer ${mvg_viewer_SRCS})
    add_dependencies(vitelotte_examples mvg_viewer)
    add_dependencies(mvg_viewer vitelottes_vg_mesh_renderer_shaders)

    target_link_libraries(mvg_viewer
        vitelotte_examples_common
        vitelotte_examples_common
        Qt5::Widgets
        Qt5::OpenGL
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
    )
else()
    message(WARNING "Skipping mvg_viewer because of the following missing dependencies:${mvg_viewer_MISSING}")
endif()

