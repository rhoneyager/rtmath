# Create the rtmath library

include (common)
include(addconfiginfo)
include(addlib)

# Setting this to prevent forced link of all symbols.
# I need this for a few builds where I am making extensive changes to the code.
#set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

include (files_src.cmake)

set (libsrcs ${srcfiles})


# Shared/static flag
if(WIN32 AND NOT CYGWIN)
	set (sflag STATIC)
else()
	set (sflag SHARED)
endif()
#find_package(Ryan_Debug REQUIRED)
#option(USE_RYAN_SERIALIZATION "Use Ryan_Serialization library in core rtmath libraries" OFF)
#set(USE_RYAN_SERIALIZATION_FLAG 0)
#if (USE_RYAN_SERIALIZATION)
#	find_package(Ryan_Serialization REQUIRED)
#	include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})
#	set(USE_RYAN_SERIALIZATION_FLAG 1)
#endif()
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../ryan-debug/)
#include_directories(${RYAN_DEBUG_INCLUDE_DIRS})

configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/rtmath/cmake-settings.h.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
	)

configure_file (
        "${CMAKE_CURRENT_SOURCE_DIR}/data/defaults/modulefiles/rtmath"
        "${CMAKE_CURRENT_BINARY_DIR}/module-rtmath"
        )

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

#set (boost_uniform_libs program_options iostreams filesystem system unit_test_framework regex date_time log log_setup)
#if (USE_RYAN_SERIALIZATION)
#	set (boost_uniform_libs ${boost_uniform_libs} serialization)
#endif()
#addBoostUniform(${boost_uniform_libs})
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

# linterp is in the related directory
include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/../related/linterp/rncarpio-linterp-ca556a0/src")

#message("${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY}")

if(WIN32 AND NOT CYGWIN)
	set(rtmath_core-libs "")
else()
	set (rtmath_core-libs ${Boost_FILESYSTEM_LIBRARY}
		${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_DATE_TIME_LIBRARY}
		${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY} pthread
		)
	if (USE_RYAN_SERIALIZATION)
		set (rtmath_core-libs ${rtmath_core-libs} ${Boost_SERIALIZATION_LIBRARY})
	endif()
endif()

add_library (rtmath_core SHARED
	${rtmath_core-files} 
	#${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h 
	${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h
	${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
	)
addlib(rtmath_core SHARED)
#add_precompiled_header(rtmath_core Stdafx-core.h src)
set (rtmath_core-libs ${rtmath_core-libs} Ryan_Debug )
#if (USE_RYAN_SERIALIZATION)
#		set (rtmath_core-libs ${rtmath_core-libs} Ryan_Serialization)
#endif()
option(USE_OLD_CPP03_ABI "C++11 led to changes in the GLIB C++ ABI. Turn this on if you are linking against old system libraries" OFF)
if(USE_OLD_CPP03_ABI)
	target_compile_definitions(rtmath_core PUBLIC _GLIBCXX_USE_CXX11_ABI=0)
endif()
target_link_libraries(rtmath_core ${rtmath_core-libs})
storebin(rtmath_core)

add_library (rtmath_ddscat_base SHARED
	${rtmath_ddscat_base-files}
	)
addlib(rtmath_ddscat_base SHARED)
set (rtmath_ddscat_base-libs 
	rtmath_core ${rtmath_core-libs} 
	)
target_link_libraries(rtmath_ddscat_base ${rtmath_ddscat_base-libs})
storebin(rtmath_ddscat_base)


add_library(rtmath_voronoi SHARED ${rtmath_voronoi-files})
addlib(rtmath_voronoi SHARED)
set (rtmath_voronoi-libs rtmath_core ${rtmath_core-libs} 
	rtmath_ddscat_base ${rtmath_ddscat_base-libs}
	#	${VTK_LIBRARIES}
	#	${VORO_LIBRARIES}
	)
	#	${QHULL_LIBRARIES}
target_link_libraries(rtmath_voronoi ${rtmath_voronoi-libs})
storebin(rtmath_voronoi)


add_library (rtmath_ddscat SHARED
	${rtmath_ddscat-files}
	)
addlib(rtmath_ddscat SHARED)
set (rtmath_ddscat-libs 
	rtmath_core ${rtmath_core-libs} 
	#	rtmath_mie ${rtmath_mie-libs}
        rtmath_ddscat_base ${rtmath_ddscat_base-libs}	
	rtmath_voronoi ${rtmath_voronoi-libs}
	)
target_link_libraries(rtmath_ddscat ${rtmath_ddscat-libs})
storebin(rtmath_ddscat)

add_library(rtmath_data SHARED ${rtmath_data-files})
addlib(rtmath_data SHARED)
set (rtmath_data-libs rtmath_core ${rtmath_core-libs}
	rtmath_voronoi ${rtmath_voronoi-libs}
	)
target_link_libraries(rtmath_data ${rtmath_data-libs})
storebin(rtmath_data)


add_library(rtmath_rt SHARED ${rtmath_rt-files})
addlib(rtmath_rt SHARED)
set (rtmath_rt-libs rtmath_core ${rtmath_core-libs}
	)
target_link_libraries(rtmath_rt ${rtmath_rt-libs})
storebin(rtmath_rt)

add_library(rtmath_images SHARED ${rtmath_images-files})
addlib(rtmath_images SHARED)
set (rtmath_images-libs rtmath_core ${rtmath_core-libs}
	rtmath_voronoi ${rtmath_voronoi-libs}
	)
target_link_libraries(rtmath_images ${rtmath_images-libs})
storebin(rtmath_images)


if (BUILD_TESTING)
	add_executable(rtmath_core_tests ${rtmath_core_test-files})
	storebin(rtmath_core_tests)

	add_executable(rtmath_ddscat_tests ${rtmath_ddscat_test-files})
	storebin(rtmath_ddscat_tests)

set_target_properties( rtmath_core_tests PROPERTIES FOLDER "Tests")
target_link_libraries(rtmath_core_tests ${rtmath_ddscat-libs} rtmath_core rtmath_mie rtmath_ddscat ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
set_target_properties( rtmath_ddscat_tests PROPERTIES FOLDER "Tests")
target_link_libraries(rtmath_ddscat_tests ${rtmath_ddscat-libs} rtmath_core rtmath_mie rtmath_ddscat ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

endif()

# Install targets. This ensures that the library is in a valid location, 
# and places the testing program in the appropriate directory.

# Data directory
if (INSTALL_DATA)
INSTALL(DIRECTORY data
	DESTINATION ${DATA_DIR_PREFIX}
	COMPONENT Data
	PATTERN ".svn" EXCLUDE
	)
endif()

# Headers
#INSTALL(DIRECTORY rtmath 
#	DESTINATION ${INSTALL_INCLUDE_DIR}
#	COMPONENT Headers
#	FILES_MATCHING PATTERN "*.h"
#	)

#IF(RTMATH-TESTS)
# Executables and libraries
#INSTALL(TARGETS rtmath-tests
#	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
#	COMPONENT Tests
#	)
#ENDIF()

#INSTALL(TARGETS rtmath_core rtmath_ddscat rtmath_ddscat_base 
#	rtmath_voronoi rtmath_data rtmath_images
	# rtmath_mie
	#	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	#	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	#	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	#	COMPONENT Libraries
	#	)

# Install modulefiles on linux / unix / mac
if (NOT WIN32)
	if (INSTALL_MODULES)
		#	INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
		#	DESTINATION ${ENV_MOD_DIR_PREFIX}
		#	COMPONENT Data
		#	RENAME ${MODULES_FILENAME}
		#	)
	endif()
endif()

