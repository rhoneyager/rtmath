cmake_minimum_required (VERSION 2.8)
project(obsSkinTemp CXX C)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(common)
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for main libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for headers")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/CMake/obsSkinTemp)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")
 
set(MAJOR 0)
set(MINOR 2)
# Get subversion revision information - for use in cpack
#EXECUTE_PROCESS("svn info . | grep Revision: |cut -c11-" 
#	OUTPUT_VARIABLE REVISION)
EXECUTE_PROCESS(COMMAND "svn" "info" "${CMAKE_CURRENT_SOURCE_DIR}"
	OUTPUT_FILE "SVN-INFO")

EXECUTE_PROCESS(COMMAND "grep" "Revis" "SVN-INFO"
	OUTPUT_FILE "SVN-REV")

EXECUTE_PROCESS(COMMAND "cut" "-c11-" "SVN-REV"
	OUTPUT_VARIABLE REVISION
	OUTPUT_STRIP_TRAILING_WHITESPACE)
	
set(VERSION
	${MAJOR}.${MINOR}.${REVISION} )
find_package(Doxygen)
option (BUILD_DOCUMENTATION
	"Build the documentation for this library" OFF)
if(BUILD_DOCUMENTATION)

	option (BUILD_DOCUMENTATION_IN_ALL
		"Build documentation automatically with 'make all'. Also used for 'make install' and 'make package'" OFF)

	if (NOT DOXYGEN_FOUND)
		message(SEND_ERROR "Documentation build requested but Doxygen is not found.")
	endif()

	set(INSTALL_DOC_DIR share/doc/obsSkinTemp-${VERSION} CACHE PATH "Installation directory for documentation")
	configure_file(Doxyfile.in
		"${PROJECT_BINARY_DIR}/Doxyfile" @ONLY)

	if (BUILD_DOCUMENTATION_IN_ALL)
		set (ALL_FLAG ALL)
	else()
		set (ALL_FLAG "")
	endif()

	# This builds the html docs
	add_custom_target(doc-html ${ALL_FLAG}
		${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
		COMMENT "Generating API html documentation with Doxygen" VERBATIM
	)
	# This builds the latex docs
	add_custom_target(doc-latex ${ALL_FLAG}
		latex refman.tex
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/docs/latex
		COMMENT "Generating API pdf documentation with Doxygen" VERBATIM
	)

	add_custom_target(docs ${ALL_FLAG} DEPENDS doc-html doc-latex)
endif()
# Enable C++11
# g++
IF(CMAKE_COMPILER_IS_GNUCXX)
	SET (COMMON_CFLAGS ${COMMON_CFLAGS} -std=c++11)
ENDIF()

IF(MSVC)
	# MSVC parallel builds by default
	if ("${COMMON_CFLAGS}" MATCHES "MP")
	else()
		SET(COMMON_CFLAGS ${COMMON_CFLAGS} /MP)
	endif()
	#add_definitions(-DCONF="$(Configuration)")
else()
	if (NOT CMAKE_BUILD_TYPE)
		#		add_definitions(-DCONF="Release")
	else()
		#		add_definitions(-DCONF="${CMAKE_BUILD_TYPE}")
	endif()
ENDIF()



# Make relative paths absolute (needed later on)
foreach(p LIB PLUGIN BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()


find_package(Boost COMPONENTS program_options 
	iostreams filesystem system REQUIRED)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
if(WIN32 AND NOT CYGWIN)
	option ( AUTOLINK_BOOST
		"Automatically link Boost" ON)
	if (AUTOLINK_BOOST)
		link_directories(${Boost_LIBRARY_DIR})
		set(libs "")
	else()
		add_definitions(-DBOOST_ALL_NO_LIB)
		set(libs ${Boost_FILESYSTEM_LIBRARY} 
		${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY})
	endif()
else()
	set (libs ${Boost_FILESYSTEM_LIBRARY} 
		${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

find_package(Ryan_Debug REQUIRED)
find_package(Ryan_Serialization REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})

find_package(silo REQUIRED)
include_directories(${SILO_INCLUDE_DIRS})

find_package(HDF5 COMPONENTS C CXX REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR_CPP})
add_definitions(${HDF5_DEFINITIONS})

if(DEFINED HDF5_C_LIBRARIES)
	set (h5libs ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
else()
	set (h5libs hdf5 hdf5_cpp)
endif()
FIND_PACKAGE(SZIP)
#	message("${SZIP_INCLUDE_DIRS}")
#	message("${SZIP_LIBRARIES}")
IF (DEFINED SZIP_LIBRARIES)
	include_directories(${SZIP_INCLUDE_DIRS})
		SET (h5libs ${h5libs} szip)
ENDIF()

add_subdirectory (../../rtmath_hdf5_cpp "${CMAKE_CURRENT_BINARY_DIR}/rtmath_hdf5_cpp")
add_subdirectory (../../rtmath_silo_cpp "${CMAKE_CURRENT_BINARY_DIR}/rtmath_silo_cpp")

add_executable (obsSkinTemp-convert 
	export.cpp
	export.h
	main.cpp
	parser.cpp
	parser.h
	observation.cpp
	observation.h
	)
target_link_libraries(obsSkinTemp-convert ${h5libs} ${libs} rtmath_hdf5_cpp Ryan_Debug Ryan_Serialization)
INSTALL(TARGETS obsSkinTemp-convert RUNTIME DESTINATION bin COMPONENT Applications)


add_executable (obsSkinTemp-silo
	import.cpp
	import.h
	main-import.cpp
	observation.cpp
	observation.h
	)
target_link_libraries(obsSkinTemp-silo ${h5libs} ${libs} rtmath_hdf5_cpp silo rtmath_silo_cpp Ryan_Debug Ryan_Serialization)
INSTALL(TARGETS obsSkinTemp-silo RUNTIME DESTINATION bin COMPONENT Applications)


