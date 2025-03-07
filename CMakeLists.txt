cmake_minimum_required(VERSION 3.10)
project(BoidsSimulation C)

set(CMAKE_C_STANDARD 99)

# Add the project root as an include directory so headers like boidUpdate.h can be found.
include_directories(${CMAKE_SOURCE_DIR})

# Build the local executable.
add_executable(local_main local_main.c boidUpdate.c)
target_link_libraries(local_main m)

# Build the distributed executable.
add_executable(distributed_main distributed_main.c boidUpdate.c messaging.c)
target_link_libraries(distributed_main m rabbitmq)
