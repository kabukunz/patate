
################################################################################
# Generate Vitelotte's shaders                                                 #
# TODO: find a more appropriate place for this.                                #
################################################################################

find_package(PythonInterp 3.0)

if(PYTHONINTERP_FOUND)
    set(vitelotte_SHADERS
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/vert_common.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/ctrl_common.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/eval_common.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/geom_common.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/frag_common.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/frag_linear.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/frag_quadratic.glsl
        ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders/frag_wireframe.glsl
    )

    add_custom_command(
        OUTPUT ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders.hpp
        COMMAND ${PYTHON_EXECUTABLE} ${Patate_SOURCE_DIR}/bin/shader2cpp
            -l
            -n Vitelotte::VGMeshRendererShaders
            -g _VITELOTTE_UTILS_VG_MESH_RENDERER_SHADERS_
            -o ${Patate_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders.hpp
            ${vitelotte_SHADERS}
        DEPENDS ${vitelotte_SHADERS} ${PROJECT_SOURCE_DIR}/bin/shader2cpp
        WORKING_DIRECTORY ${Patate_SOURCE_DIR}
        VERBATIM
    )
endif()

add_custom_target(vitelottes_vg_mesh_renderer_shaders
    DEPENDS ${PROJECT_SOURCE_DIR}/Patate/Vitelotte/Utils/vgMeshRendererShaders.hpp)


add_custom_target(vitelotte_examples)

add_subdirectory(common)
add_subdirectory(mvgtk)
add_subdirectory(mvg_viewer)
add_subdirectory(mvg_editor)
add_subdirectory(gen_mesh)

add_subdirectory(tutorial)

