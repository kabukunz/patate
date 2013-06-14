project(PatateExamples)

if(DEFINED ENV{EIGEN3_INCLUDE_DIR})
  include_directories ( "$ENV{EIGEN3_INCLUDE_DIR}" )
else()
  message("Warning: You need to set an environment variable EIGEN3_INCLUDE_DIR to be able to compile examples with your/path/to/Eigen")
endif()

include_directories(${CMAKE_SOURCE_DIR})

add_custom_target(examples)

add_subdirectory(cpp)


