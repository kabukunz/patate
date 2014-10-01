project(VitelotteExample)

cmake_minimum_required(VERSION 2.8)

################################################################################
# Vitelotte                                                                    #
################################################################################

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake ${PROJECT_SOURCE_DIR}/cmake)
find_package(X11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${GLFW3_INCLUDE_DIR})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(vitelotte_SRCS
    application.cpp 
)
add_executable(vitelotte ${vitelotte_SRCS})

message("DEBUG:" ${GLFW3_LIBRARIES})
target_link_libraries(vitelotte
	${GLFW3_LIBRARIES}
	${GLEW_LIBRARIES}
)
