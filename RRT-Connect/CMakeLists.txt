cmake_minimum_required(VERSION 3.24)
project(RRT-Connect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")

add_executable(RRT-Connect main.cpp source/RRTConnect.h source/RRTConnect.cpp)
