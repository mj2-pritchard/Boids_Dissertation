# CMake configuration file for the project.
# Sets up the build environment and defines how to compile the project into both the local and distributed implementation executable.

# Minimum required version of CMake specification. To ensure that the installed CMake supports the features used in this file.
cmake_minimum_required(VERSION 3.10)

# Project name and the programming language used specification.
project(BoidsSimulation C)

# Setting the C standard for the project to C99 standard.
set(CMAKE_C_STANDARD 99)

# Add the project root as an include directory so headers like boidUpdate.h can be found.
include_directories(${CMAKE_SOURCE_DIR})

# Build the local executable.
add_executable(local_main local_main.c boidUpdate.c)
target_link_libraries(local_main m)

# Build the distributed executable.
add_executable(distributed_main distributed_main.c boidUpdate.c messaging.c)
target_link_libraries(distributed_main m rabbitmq)
