set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Grenaille                                                                    #
################################################################################

set(grenaille_basic_SRCS 
    grenaille_basic_cpu.cpp 
)
add_executable(grenaille_basic_cpu ${grenaille_basic_SRCS})

set(grenaille_reference_SRCS 
    grenaille_reference.cpp 
)
add_executable(grenaille_reference ${grenaille_reference_SRCS})
add_dependencies(examples grenaille_reference)
