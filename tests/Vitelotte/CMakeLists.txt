set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Vitelotte                                                                    #
################################################################################

add_multi_test(element.cpp)

#############################################
#set(fv_SRCS
#    fv.cpp
#)
#add_executable(fv ${fv_SRCS})


#############################################
#set(surface_mesh_SRCS
#    surface_mesh.cpp
#)
#add_executable(surface_mesh ${surface_mesh_SRCS})
#add_dependencies(buildtests deta_orthogonal_derivatives)
#add_test(deta_orthogonal_derivatives deta_orthogonal_derivatives)

#############################################
#set(fem_mesh_SRCS
#    fem_mesh.cpp
#)
#add_executable(fem_mesh ${fem_mesh_SRCS})
#add_dependencies(buildtests deta_orthogonal_derivatives)
#add_test(deta_orthogonal_derivatives deta_orthogonal_derivatives)
