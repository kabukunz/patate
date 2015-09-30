###############################################################################
# Vitelotte mvgtk                                                             #
###############################################################################

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    "../../.."
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(vitelotte_gen_mesh_SRCS
    triangle.c
    pugixml.cpp
    gen_mesh.cpp
)
add_definitions(-DNO_TIMER -DLINUX -DTRILIBRARY -DANSI_DECLARATORS)
add_executable(gen_mesh ${vitelotte_gen_mesh_SRCS})
add_dependencies(vitelotte_examples gen_mesh)

#target_link_libraries(gen_mesh
#    vitelotte_examples_common
#)

