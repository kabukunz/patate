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
set(quadratic_mesh_SRCS
    quadratic_mesh.cpp
)
add_executable(quadratic_mesh ${quadratic_mesh_SRCS})
#add_dependencies(buildtests deta_orthogonal_derivatives)
#add_test(deta_orthogonal_derivatives deta_orthogonal_derivatives)

#############################################
set(fv_mesh_SRCS
    fv_mesh.cpp
)
add_executable(fv_mesh ${fv_mesh_SRCS})
#add_dependencies(buildtests deta_orthogonal_derivatives)
#add_test(deta_orthogonal_derivatives deta_orthogonal_derivatives)
