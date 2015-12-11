project(Patate-common)
cmake_minimum_required(VERSION 2.8)


FILE ( GLOB COMMON_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/defines.h
  ${CMAKE_CURRENT_SOURCE_DIR}/gl_utils/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/surface_mesh/*.h
)

FILE ( GLOB COMMON_IMPL
  ${CMAKE_CURRENT_SOURCE_DIR}/gl_utils/*.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/surface_mesh/*.hpp
)

add_library (PATATE_COMMON OBJECT ${COMMON_HEADERS} ${COMMON_IMPL})
