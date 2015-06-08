project(VitelotteExamples)

cmake_minimum_required(VERSION 2.8)

add_custom_target(vitelotte_examples)

add_subdirectory(common)
add_subdirectory(mvgtk)
add_subdirectory(mvg_viewer)
add_subdirectory(mvg_editor)

add_subdirectory(tutorial)

