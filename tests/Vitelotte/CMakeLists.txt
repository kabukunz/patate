set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Vitelotte                                                                    #
################################################################################

#############################################
set(surface_mesh_SRCS
    surface_mesh.cpp
)
add_executable(surface_mesh ${surface_mesh_SRCS})
#add_dependencies(buildtests deta_orthogonal_derivatives)
#add_test(deta_orthogonal_derivatives deta_orthogonal_derivatives)

#############################################
set(element_mesh_SRCS
    element_mesh.cpp
)
add_executable(element_mesh ${element_mesh_SRCS})
#add_dependencies(buildtests deta_orthogonal_derivatives)
#add_test(deta_orthogonal_derivatives deta_orthogonal_derivatives)
