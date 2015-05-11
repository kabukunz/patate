###############################################################################
# Vitelotte mvgtk                                                             #
###############################################################################

project(VitelotteTutorial)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    "../../.."
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(vitelotte_tutorial_SRCS
    tutorial.cpp
)
add_executable(tutorial ${vitelotte_tutorial_SRCS})
add_dependencies(examples tutorial)
