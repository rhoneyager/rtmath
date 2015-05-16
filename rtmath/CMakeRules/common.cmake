# Common include file for all rtmath-derived projects that use CMake.

# Loads the necessary components for libraries and sets various 
# compiler flags. 

include (addapp)
include (addlib)
include (AddPackageDependency)
include (PrecompiledHeader)
include (signing)
include (generaterc)
#include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
#set (COMMON_LIBS ${COMMON_LIBS} ${ROOT_LIBRARIES})
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_Debug ${CMAKE_BINARY_DIR}/Debug)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_Release ${CMAKE_BINARY_DIR}/Release)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MinSizeRel ${CMAKE_BINARY_DIR}/MinSizeRel)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RelWithDebInfo ${CMAKE_BINARY_DIR}/RelWithDebInfo)

