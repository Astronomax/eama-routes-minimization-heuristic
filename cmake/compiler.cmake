include(cmake/utils.cmake)

if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
# Remove VALGRIND code and assertions in *any* type of release build.
    add_definitions("-DNDEBUG" "-DNVALGRIND")
endif()

#
# Perform build type specific configuration.
#

set (CMAKE_C_FLAGS_DEBUG
    "${CMAKE_C_FLAGS_DEBUG} -O0")
set (CMAKE_C_FLAGS_RELWITHDEBINFO
    "${CMAKE_C_FLAGS_RELWITHDEBINFO} -O2")
set (CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}-O2")

