########################################################################
# Vitelotte Examples common                                            #
########################################################################

project(VitelotteExamplesCommon)


set(vitelotte_examples_SHADERS
    shaders/vert_points.glsl
    shaders/geom_points.glsl
    shaders/frag_points.glsl
    shaders/vert_lines.glsl
    shaders/geom_lines.glsl
    shaders/frag_lines.glsl
)

add_custom_command(
    OUTPUT ${PROJECT_SOURCE_DIR}/shaders.cpp
    COMMAND python3 ${PROJECT_SOURCE_DIR}/../../../bin/shader2cpp -o ${PROJECT_SOURCE_DIR}/shaders.cpp -e -l ${vitelotte_examples_SHADERS}
    DEPENDS ${vitelotte_examples_SHADERS}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    VERBATIM
)

set(vitelotte_examples_common_SRCS
    glLineRenderer.cpp
    glPointRenderer.cpp
    orthographicCamera.cpp
    shaders.cpp
    textFormatter.cpp
    trackball.cpp
    vgNodeRenderer.cpp
)

add_library(vitelotte_examples_common
    ${vitelotte_examples_common_SRCS}
)
add_dependencies(examples vitelotte_examples_common)
add_dependencies(vitelotte_examples_common vitelottes_vg_mesh_renderer_shaders)
