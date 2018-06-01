project(Shelves)
cmake_minimum_required(VERSION 2.8)


FILE ( GLOB SHELVES_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)

FILE ( GLOB SHELVES_IMPL
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
)

message("ROTO : ${PATATE_ROOT}")
add_library (Shelves OBJECT ${SHELVES_HEADERS} ${SHELVES_IMPL})
set_target_properties(Shelves PROPERTIES LINKER_LANGUAGE C++)

target_include_directories(Shelves PUBLIC
  $<BUILD_INTERFACE:${PATATE_ROOT}>
  $<INSTALL_INTERFACE:include>  # <prefix>/include/mylib
)
