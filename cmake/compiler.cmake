include(cmake/utils.cmake)

if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
# Remove VALGRIND code and assertions in *any* type of release build.
    add_definitions("-DNDEBUG" "-DNVALGRIND")
endif()

#
# Perform build type specific configuration.
#
#check_c_compiler_flag("-ggdb" CC_HAS_GGDB)
#if (CC_HAS_GGDB)
    set (CC_DEBUG_OPT "-ggdb")
#endif()

set (CMAKE_C_FLAGS_DEBUG
        "${CMAKE_C_FLAGS_DEBUG} ${CC_DEBUG_OPT} -O0 -fsanitize=undefined")
set (CMAKE_C_FLAGS_RELWITHDEBINFO
        "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${CC_DEBUG_OPT} -O2")
set (CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} ${CC_DEBUG_OPT} -O0 -fsanitize=undefined")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO
        "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CC_DEBUG_OPT} -O2")

unset(CC_DEBUG_OPT)

#set(CMAKE_C_FLAGS_INIT "-fsanitize=undefined")
#set(CMAKE_CXX_FLAGS_INIT "-fsanitize=undefined")