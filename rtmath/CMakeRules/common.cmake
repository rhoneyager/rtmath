# Common include file for all rtmath-derived projects that use CMake.

# Loads the necessary components for libraries and sets various 
# compiler flags. 

include (addapp)
include (addlib)
include (AddPackageDependency)
include (PrecompiledHeader)
include (signing)
#include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
#set (COMMON_LIBS ${COMMON_LIBS} ${ROOT_LIBRARIES})

