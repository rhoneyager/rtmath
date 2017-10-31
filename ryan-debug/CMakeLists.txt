# CMake file for Ryan's Debugging library
# Designed to build a shared library (though static is also okay)

# Will handle easy project imports

set(RDMAJOR 0)
set(RDMINOR 11)
set(RDREVISION 1)

set(RYAN_DEBUG_VERSION
	${RDMAJOR}.${RDMINOR}.${RDREVISION})

configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake-settings.h.in"
	"${CMAKE_BINARY_DIR}/Ryan_Debug_cmake-settings.h"
	)
	
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/Ryan_Debug.xml.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
	)

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

set( srcs
	src/config.cpp
	src/debug.cpp
	src/dlls.cpp
	src/error.cpp
	src/fs.cpp
	src/hash.cpp
	src/info.cpp
	src/io.cpp
	src/logging.cpp
	src/macros.cpp
	src/modules.cpp
	src/MurmurHash3.cpp
	src/os_functions.cpp
	src/registry.cpp
	src/serialization.cpp
	src/splitSet.cpp
	)
set (hdrs
	Ryan_Debug/config.h
	Ryan_Debug/defs.h
	Ryan_Debug/debug.h
	Ryan_Debug/dlls.h
	Ryan_Debug/error.h
	Ryan_Debug/fs.h
	Ryan_Debug/hash.h
	Ryan_Debug/info.h
	Ryan_Debug/io.h
	Ryan_Debug/logging.h
	Ryan_Debug/macros.h
	Ryan_Debug/modules.h
	Ryan_Debug/MurmurHash3.h
	Ryan_Debug/options.h
	Ryan_Debug/plugin.h
	Ryan_Debug/registry.h
	Ryan_Debug/Serialization.h
	Ryan_Debug/splitSet.h
	src/internal.h
	#	"${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h"
	"${CMAKE_BINARY_DIR}/Ryan_Debug_cmake-settings.h"
	"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
	)
source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})
add_library(Ryan_Debug SHARED ${srcs} ${hdrs} 
	)
target_link_libraries(Ryan_Debug ${liblist})
addlib(Ryan_Debug SHARED)

add_subdirectory(plugins)
add_subdirectory(apps)


