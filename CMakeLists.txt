# Authors: Elena Ceseracciu, Luca Tagliapietra, Monica Reggiani

cmake_minimum_required(VERSION 2.8) # TODO: check if higher is needed

# Define project
project (DoubleLoopControllers)

#if(UNIX)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -Wno-deprecated-declarations ")
#endif(UNIX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# find dependencies (Opensim)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(OpenSim REQUIRED)

add_subdirectory(lib)
add_subdirectory(src)





