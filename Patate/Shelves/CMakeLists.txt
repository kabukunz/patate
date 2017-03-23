project(Shelves)
cmake_minimum_required(VERSION 2.8)


FILE ( GLOB SHELVES_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/*.h
)

FILE ( GLOB SHELVES_IMPL
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/*.hpp
)

add_library (Shelves OBJECT ${SHELVES_HEADERS} ${SHELVES_IMPL})
set_target_properties(Shelves PROPERTIES LINKER_LANGUAGE C++)
