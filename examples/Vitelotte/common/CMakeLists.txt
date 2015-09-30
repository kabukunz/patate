########################################################################
# Vitelotte Examples common                                            #
########################################################################

find_package(GLEW)

find_package(PythonInterp 3.0)

include_directories(
    ${GLEW_INCLUDE_DIRS}
)

set(vitelotte_examples_common_SRCS
    orthographicCamera.cpp
    textFormatter.cpp
    trackball.cpp
)

if(GLEW_FOUND)
    if(PYTHONINTERP_FOUND)
        set(vitelotte_examples_SHADERS
            ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders/vert_points.glsl
            ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders/geom_points.glsl
            ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders/frag_points.glsl
            ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders/vert_lines.glsl
            ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders/geom_lines.glsl
            ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders/frag_lines.glsl
        )

        add_custom_command(
            OUTPUT ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders.cpp
                   ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders.h
            COMMAND ${PYTHON_EXECUTABLE} ${Patate_SOURCE_DIR}/bin/shader2cpp -o ${PROJECT_SOURCE_DIR}/Vitelotte/common/shaders.cpp -e -l ${vitelotte_examples_SHADERS}
            DEPENDS ${vitelotte_examples_SHADERS} ${Patate_SOURCE_DIR}/bin/shader2cpp
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            VERBATIM
        )
    endif()

    set(vitelotte_examples_common_SRCS
        ${vitelotte_examples_common_SRCS}
        glLineRenderer.cpp
        glPointRenderer.cpp
        shaders.cpp
        vgNodeRenderer.cpp
    )
endif()

add_library(vitelotte_examples_common
    ${vitelotte_examples_common_SRCS}
)
add_dependencies(vitelotte_examples vitelotte_examples_common)
add_dependencies(vitelotte_examples_common vitelottes_vg_mesh_renderer_shaders)
