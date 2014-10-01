project(VitelotteExample)

cmake_minimum_required(VERSION 2.8)

################################################################################
# Vitelotte                                                                    #
################################################################################

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(vitelotte_SRCS
    application.cpp 
)
add_executable(vitelotte ${vitelotte_SRCS})

target_link_libraries(vitelotte glfw3 /usr/lib64/libGLEW.a;/usr/lib/x86_64-linux-gnu/libX11.so;-lpthread;/usr/lib/x86_64-linux-gnu/libXrandr.so;/usr/lib/x86_64-linux-gnu/libXi.so;/usr/lib/x86_64-linux-gnu/libXxf86vm.so;/usr/lib/x86_64-linux-gnu/librt.so;/usr/lib/x86_64-linux-gnu/libm.so;/usr/lib/x86_64-linux-gnu/libGL.so)
