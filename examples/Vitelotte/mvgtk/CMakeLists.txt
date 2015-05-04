###############################################################################
# Vitelotte mvgtk                                                             #
###############################################################################

project(VitelotteMvgTk)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    "../../.."
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(vitelotte_mvgtk_SRCS
    solveCommand.cpp
    mvgtk.cpp
    checkCommand.cpp
    compactCommand.cpp
    convertCommand.cpp
    curvesToNodesCommand.cpp
    finalizeCommand.cpp
    plotCommand.cpp
    simplifyCommand.cpp
    main.cpp
)
add_executable(mvgtk ${vitelotte_mvgtk_SRCS})
add_dependencies(examples mvgtk)

target_link_libraries(mvgtk
    vitelotte_examples_common
)
