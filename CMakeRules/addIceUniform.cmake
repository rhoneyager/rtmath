macro(addIceUniform )
	#set(Ice_DEBUG ON)
#set(BOOST_ALL_DYN_LINK ON)

if (WIN32 AND NOT CYGWIN)
	set(IceLibPrefix "")
	set(IceLibSuffix .lib)
	set(IceExeSuffix .exe)
else()
	set(IceLibPrefix lib)
	set(IceLibSuffix .so)
	set(IceExeSuffix "")
endif()
#find_package(Ice REQUIRED)
#if (Ice_LIBRARIES)
#	set(liblist ${liblist} ${Ice_LIBRARIES})
#endif()

include_directories(BEFORE SYSTEM ${Ice_INCLUDE_DIR})
#message("ild ${ICE_LIBRARY_DIR_HINT_PATH}")
#message("ibd ${ICE_BIN_DIR_HINT_PATH}")
#link_directories(${Ice_LIBRARY_DIR})
#message("argv ${ARGV} ${args}")

set(_ICE_SEARCHES)
list(APPEND _ICE_SEARCHES ICE_LIBRARY_DIR_HINT_PATH)

set(ICE_LIBRARIES)

foreach(l ${ARGV})
	#message("Adding ${IceLibPrefix}${l}${IceLibSuffix}")
	
	find_library(ICE_${l}_LIBRARY_RELEASE NAMES ${IceLibPrefix}${l}${IceLibSuffix} PATHS ${ICE_LIBRARY_DIR_HINT_PATH} )
	mark_as_advanced(ICE_${l}_LIBRARY_RELEASE)
	find_library(ICE_${l}_LIBRARY_DEBUG NAMES ${IceLibPrefix}${l}d${IceLibSuffix} PATHS ${ICE_LIBRARY_DIR_HINT_PATH} )
	mark_as_advanced(ICE_${l}_LIBRARY_DEBUG)

	#message("Debug: ${ICE_${l}_LIBRARY_DEBUG}")
	#message("Release: ${ICE_${l}_LIBRARY_RELEASE}")

	include(${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake)
	SELECT_LIBRARY_CONFIGURATIONS(ICE_${l})
	#message("${ICE_${l}_LIBRARIES}")
	list(APPEND ICE_LIBRARIES ${ICE_${l}_LIBRARIES})

endforeach()
#message("IL ${ICE_LIBRARIES}")

# Also install / pack the relevant boost libraries into the install tree
# Do extention swapping
#if(WIN32)
#	set(dllext ".dll")
#	set(libext "[.]lib")
#else()
#	set(dllext ".so")
#	set(libext "[.]a")
#endif()


endmacro(addIceUniform args)

