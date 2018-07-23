# CMake solution for the tmatrix library
# This conveniently can produce unix makefiles, eclipse .project files, 
# visual studio solutions and xcode projects

cmake_minimum_required (VERSION 2.8)
project(tmatrix CXX Fortran C)
#project(tmatrix CXX C)

include (CTest)
enable_testing()


# Add path for custom modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/../../CMakeRules/"
)

include(addBaseProject)
include(addlib)
include(addAssembly)
include(addconfiginfo)
include(addInstallDirs)
include(addModuleFiles)
include(addBoostUniform)
include(adddocs)
include(generateAssemblyGuid)

set(MAJOR 0)
set(MINOR 7)
set(REVISION 2)
set(SVNREVISION 1)
set(TMATRIX_VERSION ${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION})
math(EXPR PREVSVNREVISION "${SVNREVISION} - 1")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

addBaseProject()
addInstallDirs(Tmatrix ${TMATRIX_VERSION})
addModuleFiles(Tmatrix ${TMATRIX_VERSION})
addAssembly(Tmatrix) # ${MAJOR} ${MINOR} ${REVISION} ${SVNREVISION} ${INSTALL_BIN_DIR}

set(BUILD_CPP on)

if (BUILD_CPP)
	#find_package(Ryan_Debug REQUIRED)
	#find_package(Ryan_Serialization REQUIRED)
	#set(liblist Ryan_Serialization Ryan_Debug)
	#addBoostUniform(serialization system)
	set(liblist Ryan_Debug)
	addBoostUniform(program_options filesystem system)

	include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
	#link_directories(${Boost_LIBRARY_DIR})
	
	if (WIN32 AND NOT CYGWIN)
	else()
		set(liblist ${liblist} pthread)
	endif()
endif()

option ( BUILD_APPS
	"Build the default applications" OFF)



# The target processing
# Recurse into the subdirectories
add_subdirectory (src)

if (BUILD_APPS AND BUILD_CPP)
	add_subdirectory (apps)
endif()

adddocs()

