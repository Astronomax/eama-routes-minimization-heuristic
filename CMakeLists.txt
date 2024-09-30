cmake_minimum_required(VERSION 3.26)
project(routes C CXX ASM)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_INCLUDE_PATH})

set(CMAKE_C_STANDARD 11)

include(cmake/utils.cmake)
include(cmake/compiler.cmake)
include(cmake/SetFiberStackSize.cmake)

#
# LibCORO
#
include(BuildLibCORO)
libcoro_build()

#
# Specify where to look for include files.
#
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/lib)
include_directories(${PROJECT_SOURCE_DIR}/src/lib/small_extra/include)
include_directories(${PROJECT_BINARY_DIR}/src/lib/small_extra/small_extra/include)
include_directories(${PROJECT_SOURCE_DIR}/src/lib/small/include)
include_directories(${PROJECT_BINARY_DIR}/src/lib/small/small/include)
include_directories(${PROJECT_SOURCE_DIR}/src/lib/small/third_party)
include_directories(${PROJECT_SOURCE_DIR}/src/lib/core)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

enable_testing()

add_subdirectory(src)

set (sources
    src/c_penalty.c
    src/cli.c
    src/customer.c
    src/distance.c
    src/eama_solver.c
    src/ejection.c
    src/modification.c
    src/problem.c
    src/problem_decode.cc
    src/random_utils.c
    src/route.c
    src/solution.cc
    src/solution_encode.cc
    src/tw_penalty.c
    src/main.c
)

add_executable(routes ${sources})
target_compile_options(routes PRIVATE -Wall -Wextra -Wpedantic -Wno-gnu-statement-expression)
target_link_libraries(routes small core)

add_subdirectory(test)
