cmake_minimum_required(VERSION 3.16)

# Activer le support des symboles de débogage
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(project-name)
