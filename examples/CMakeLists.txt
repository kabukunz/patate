project(PatateExamples)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_SOURCE_DIR})

add_custom_target(examples)

add_subdirectory(cpp)


