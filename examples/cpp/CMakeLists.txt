set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Grenaille                                                                    #
################################################################################

set(grenaille_basic_SRCS 
    grenaille_basic_cpu.cpp 
)
add_executable(grenaille_basic_cpu ${grenaille_basic_SRCS})

set(grenaille_binding_SRCS 
    grenaille_binding.cpp 
)
add_executable(grenaille_binding ${grenaille_binding_SRCS})
add_dependencies(examples grenaille_binding)
