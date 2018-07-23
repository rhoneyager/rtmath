# Load some basic macros which are needed later on
include (common)

#option (BUILD_CPP "Build the C++ side of the library" ON)
#option (BUILD_FORTRAN "Build the Fortran library" ON)
set (BUILD_CPP ON)
set (BUILD_FORTRAN ON)

# Select C++ and Fortran sources
# Intel Fortran on Windows requires separate logical projects for the C++ and Fortran code
set (srcs_fortran fortran/tmd.lp.f fortran/lpd.f )
set (srcs_fortran_iso fortran/lpd.f fortran/tmd.lp.f)
set (srcs_cpp cpp/tmatrix.cpp ) # cpp/tmatrix-serialization.cpp )
set (srcs_tests tests/main.cpp )
set (hdrs_for fortran/common.defs fortran/description.txt )
set (hdrs_for_iso fortran/common_iso.defs)
set (hdrs_cpp
	headers/par.h
	headers/fortrancommon.h
	headers/tmatrix.h
	headers/defs.h
	#headers/tmatrix-serialization.h
	)
source_group("Header Files" FILES ${hdrs_for} ${hdrs_cpp})

if (WIN32 AND NOT CYGWIN)
	configure_file(resource.h.in 
		"${PROJECT_BINARY_DIR}/resource.h" @ONLY)
endif()
add_definitions(-DWITH_CMAKE)

configure_file (
        "${CMAKE_CURRENT_SOURCE_DIR}/headers/cmake-settings.h.in"
        "${PROJECT_BINARY_DIR}/headers/cmake-settings.h"
        )
configure_file (
        "${CMAKE_CURRENT_SOURCE_DIR}/environment/tmatrix/tmatrix"
        "${PROJECT_BINARY_DIR}/module-tmatrix"
        )


enable_language(Fortran)
if (WIN32 AND NOT CYGWIN)
	
	# Fortran needs more work
	if (CMAKE_Fortran_COMPILER MATCHES "[^p]gfortran")
		# Note: [^p] because otherwise pgfortran.exe would match this.
		# With gfortran, need to first preprocess the file, expand the CONF macro, and convert to an object file.
		configure_file(tmatrixfor.rc.in
			"${PROJECT_BINARY_DIR}/tmatrixfor.rc" @ONLY NEWLINE_STYLE WIN32)
		add_custom_command(
			OUTPUT "${PROJECT_BINARY_DIR}/tmatrixfor.o"
			COMMAND ${CMAKE_RC_COMPILER} 
			"${PROJECT_BINARY_DIR}/tmatrixfor.rc" 
			"${PROJECT_BINARY_DIR}/tmatrixfor.o"
			DEPENDS
			"${PROJECT_BINARY_DIR}/tmatrixfor.rc"
			)	
		set(srcs_fortran ${srcs_fortran} "${PROJECT_BINARY_DIR}/tmatrixfor.o" "${PROJECT_BINARY_DIR}/resource.h" )
	else()
		# Using devenv with ifort and RC files is annoying.
		# CMake wants to force the project type to 
		# be a C++ project. So, I have to compile the 
		# rc file in a separate step.
		if (CMAKE_Fortran_COMPILER MATCHES "devenv|ifort")
			# Gets set in compiler defines
			#set(CONF "$(Configuration)\"${configappend}\"")
			#set(CONF "\"${configappend}\"")
		else()
			set(CONF "\"${CMAKE_BUILD_TYPE}\"") 
			# Outside of devenv.exe is an nmake build
		endif()

		configure_file(tmatrixfor.rc.in
			"${PROJECT_BINARY_DIR}/tmatrixfor.rc" @ONLY NEWLINE_STYLE WIN32)
		set_source_files_properties("${PROJECT_BINARY_DIR}/tmatrixfor.rc" PROPERTIES LANGUAGE RC)	
		add_library (tmatrix_random_fortran-rcs OBJECT  "${PROJECT_BINARY_DIR}/tmatrixfor.rc") #cpp/null.cpp)
		
		set_target_properties(tmatrix_random_fortran-rcs PROPERTIES COMPILE_DEFINITIONS 
			"RCCONF=\"$<CONFIGURATION>${configappend}\"")
		#get_target_property(rcsCD tmatrix_random_fortran-rcs COMPILE_DEFINITIONS)
		#message(${rcsCD})
		SET_TARGET_PROPERTIES(tmatrix_random_fortran-rcs PROPERTIES LINKER_LANGUAGE Fortran)
		set (TMATRIX_FORTRAN_LINK_RC_INTEL ON)
		set (srcs_fortran ${srcs_fortran} $<TARGET_OBJECTS:tmatrix_random_fortran-rcs>)
		set_target_properties(tmatrix_random_fortran-rcs PROPERTIES FOLDER "Libs")

	endif()
	#set(srcs_fortran ${srcs_fortran} "${PROJECT_BINARY_DIR}/tmatrixfor.o" "${PROJECT_BINARY_DIR}/resource.h" )
endif()

add_library (tmatrix_random_fortran SHARED ${srcs_fortran} ${hdrs_for})
#$<TARGET_OBJECTS:tmatrix_random_fortran-rcs> ) # Set above
if (DEFINED TMATRIX_FORTRAN_LINK_RC_INTEL)
	add_custom_command(TARGET tmatrix_random_fortran PRE_LINK
		COMMAND copy 
		$<CONFIGURATION>\\tmatrixfor.res
		$<CONFIGURATION>\\tmatrixfor.obj
		WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/src/tmatrix_fortran-rcs.dir/"
		COMMENT "Copying rc / res file to proper extension")
	add_dependencies(tmatrix_random_fortran tmatrix_random_fortran-rcs)
endif()

set_target_properties(tmatrix_random_fortran PROPERTIES LINKER_LANGUAGE Fortran)
addlib(tmatrix_random_fortran SHARED)
signing(tmatrix_random_fortran)

if (CMAKE_Fortran_COMPILER MATCHES "[^p]gfortran")
	# Set some gfortran-specific options
	set_target_properties(tmatrix_random_fortran PROPERTIES COMPILE_FLAGS ${COMPILE_FLAGS} -cpp)
endif()

if (CMAKE_Fortran_COMPILER MATCHES "pgfortran")
	# Doesn't work yet --- still emits -shared
	# PGI build also doesn't save .lib files.
	set(CMAKE_SHARED_LIBRARY_CREATE_${lang}_FLAGS "-Mmakedll")
endif()

if (WIN32 AND NOT CYGWIN)
	configure_file(tmatrix.rc.in
		"${PROJECT_BINARY_DIR}/tmatrix.rc" @ONLY NEWLINE_STYLE WIN32)
	set(srcs_cpp ${srcs_cpp} "${PROJECT_BINARY_DIR}/tmatrix.rc" "${PROJECT_BINARY_DIR}/resource.h")
endif()

add_library (tmatrix_random_cpp SHARED ${srcs_cpp} ${hdrs_cpp}
	${PROJECT_BINARY_DIR}/headers/cmake-settings.h
	${PROJECT_BINARY_DIR}/module-tmatrix
	)
addlib(tmatrix_random_cpp SHARED)
if (NOT DEFINED MAKE_ASSEMBLY)
	signing(tmatrix_random_cpp) # Will be part of the assembly code
endif()
get_target_property(tmatrix_random_cpp_INCLUDE_DIRS tmatrix_random_cpp INCLUDE_DIRECTORIES)
if (NOT tmatrix_random_cpp_INCLUDE_DIRS)
	set(tmatrix_random_cpp_INCLUDE_DIRS "")
endif()
set(tmatrix_cpp_LIBRARIES "")
include_directories (${RYAN_DEBUG_INCLUDE_DIRS} ) #${RYAN_SERIALIZATION_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../headers ${CMAKE_CURRENT_BINARY_DIR}/../ 
	${CMAKE_CURRENT_BINARY_DIR}/headers)
target_link_libraries (tmatrix_random_cpp ${liblist})
add_dependencies (tmatrix_random_cpp tmatrix_random_fortran )
target_link_libraries (tmatrix_random_cpp tmatrix_random_fortran ${liblist})
set_target_properties(tmatrix_random_fortran PROPERTIES Fortran_FORMAT FREE)


if (MAKE_ASSEMBLY)
	foreach(p Debug RelWithDebInfo MinSizeRel Release)
		set(ASSEMBLY_NAME ${ASSEMBLY_NAME_${p}})
		set(ASSEMBLY_FORTRAN_LIB_NAME tmatrix_random_fortran_${p}${configappend}.dll)
		set(ASSEMBLY_CPP_LIB_NAME tmatrix_random_cpp_${p}${configappend}.dll)
		#if (${p} STREQUAL Debug)
		#	set(ASSEMBLY_FORTRAN_IFCORE libifcoremdd.dll)
		#	set(ASSEMBLY_FORTRAN_MMD libmmdd.dll)
		#else()
		#	set(ASSEMBLY_FORTRAN_IFCORE libifcoremd.dll)
		#	set(ASSEMBLY_FORTRAN_MMD libmmd.dll)
		#endif()
		generateAssemblyGuid(TmatrixRandom ${p} policy sxspolicyguid)
		generateAssemblyGuid(TmatrixRandom ${p} public sxspublicguid)
		configure_file(
			"Ryan.TmatrixRandom.Release.manifest.template"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/${ASSEMBLY_NAME_${p}}.pre.manifest" @ONLY)
		configure_file(
			"${CMAKE_CURRENT_SOURCE_DIR}/sxs.template.wxs"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/sxs_${ASSEMBLY_NAME_${p}}.wxs" @ONLY)	
	endforeach()
	implementAssembly(Tmatrix tmatrix_random_cpp )

	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/config${configappend}.wxs" @ONLY)
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Ryan_TmatrixRandom.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/TmatrixRandom.wxs" @ONLY)
	INSTALL(FILES
		"${CMAKE_CURRENT_BINARY_DIR}/config${configappend}.wxs"
		DESTINATION "${INSTALL_INCLUDE_DIR}/../installer" COMPONENT Scripts)
	INSTALL(FILES
		"${CMAKE_CURRENT_BINARY_DIR}/TmatrixRandom.wxs"
		"${CMAKE_CURRENT_SOURCE_DIR}/upgrade.bat"
		"${CMAKE_CURRENT_SOURCE_DIR}/collectdirs.bat"
		"${CMAKE_CURRENT_SOURCE_DIR}/deploy.bat"
		"${CMAKE_CURRENT_SOURCE_DIR}/license.rtf"
		DESTINATION "${INSTALL_INCLUDE_DIR}/.." COMPONENT Scripts)
endif()


if (BUILD_TESTING)
	if (WIN32 AND NOT CYGWIN)
		configure_file(tmatrixTests.rc.in
			"${PROJECT_BINARY_DIR}/tmatrixTests.rc" @ONLY NEWLINE_STYLE WIN32)
		set(srcs_tests ${srcs_tests} "${PROJECT_BINARY_DIR}/tmatrixTests.rc" "${PROJECT_BINARY_DIR}/resource.h")
	endif()
	add_executable (tmatrix-random-tests ${srcs_tests} )
	addconfiginfo(tmatrix-random-tests)
	add_dependencies (tmatrix-random-tests tmatrix_random_cpp tmatrix_random_fortran)
	target_link_libraries (tmatrix-random-tests tmatrix_random_cpp tmatrix_random_fortran ${liblist})
	set_target_properties (tmatrix-random-tests PROPERTIES FOLDER "Tests")
	IF(DEFINED COMMON_CFLAGS )
		set_target_properties(tmatrix-random-tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	endif()
	if (MAKE_ASSEMBLY)
		signing(tmatrix-random-tests ) # Will be part of the assembly code
	endif()

endif()

# Add include directory of fortran source to allow fortran #include statements to work
include_directories (AFTER 
	${CMAKE_CURRENT_SOURCE_DIR}/fortran
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_BINARY_DIR}
	)

# Disable useless warnings
# TODO: fix this!
#if ("${CMAKE_C_COMPILER}" MATCHES "i.*$") 
#	set_target_properties(tmatrix_fortran PROPERTIES COMPILE_FLAGS /Qdiag-disable:8290,8291)
#endif()

set (tmcpptargets tmatrix_random_cpp tmatrix_random_fortran )
set (tmfortrantargets tmatrix_random_fortran )

export(TARGETS tmatrix_random_cpp tmatrix_random_fortran 
	FILE "${PROJECT_BINARY_DIR}/TmatrixRandomTargets.cmake")

set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${PROJECT_SOURCE_DIR}/TmatrixRandomConfig.cmake.in
	"${PROJECT_BINARY_DIR}/TmatrixRandomConfig.cmake" @ONLY)
# ${TMATRIX_CPP_CMAKE_DIR} is set in TmatrixConfig.cmake.in - note
# the \${ in the variable below.
set(CONF_INCLUDE_DIRS "\${TMATRIX_RANDOM_CPP_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${PROJECT_SOURCE_DIR}/TmatrixRandomConfig.cmake.in
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/TmatrixRandomConfig.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/TmatrixRandomConfigVersion.cmake.in
	"${PROJECT_BINARY_DIR}/TmatrixRandomConfigVersion.cmake" @ONLY)


INSTALL(TARGETS ${tmcpptargets}
	EXPORT TmatrixRandomTargets
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	COMPONENT Libraries
	)
# On Windows MSVC, also copy pdb files for Debug and RelWithDebInfo
if(WIN32 AND NOT CYGWIN)
foreach(p Debug RelWithDebInfo )
	set(ASSEMBLY_NAME ${ASSEMBLY_NAME_${p}})
	set(ASSEMBLY_FORTRAN_LIB_NAME tmatrix_random_fortran_${p}${configappend}.pdb)
	set(ASSEMBLY_CPP_LIB_NAME tmatrix_random_cpp_${p}${configappend}.pdb)

	INSTALL(FILES 
		"${CMAKE_CURRENT_BINARY_DIR}/${p}/${ASSEMBLY_FORTRAN_LIB_NAME}"
		"${CMAKE_CURRENT_BINARY_DIR}/${p}/${ASSEMBLY_CPP_LIB_NAME}"
		DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
		COMPONENT Debugging
		CONFIGURATIONS ${p}
		)
endforeach()
endif()

# Duplicate for NSIS bug
if(WIN32 AND NOT CYGWIN)
	INSTALL(TARGETS tmatrix_random_fortran tmatrix_random_cpp 
		RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
		COMPONENT Libraries
		)
endif()

INSTALL(DIRECTORY headers/
	DESTINATION ${INSTALL_INCLUDE_DIR}/tmatrixRandom
	COMPONENT Headers
	FILES_MATCHING PATTERN "*.h"
	)

INSTALL(DIRECTORY cpp fortran tests
	DESTINATION ${INSTALL_INCLUDE_DIR}/../src
	COMPONENT Sources
	)


INSTALL(FILES "${PROJECT_BINARY_DIR}/headers/cmake-settings.h"
	DESTINATION ${INSTALL_INCLUDE_DIR}/tmatrixRandom
	COMPONENT Headers
	)

IF(INSTALL_MODULES AND NOT WIN32)
	INSTALL(
		FILES ${PROJECT_BINARY_DIR}/module-tmatrix
		DESTINATION ${ENV_MOD_DIR_PREFIX}
		COMPONENT Environment
		RENAME ${MODULES_FILENAME}
	)
ENDIF()

if(BUILD_TESTING)
	INSTALL(TARGETS tmatrix-random-tests
		RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
		COMPONENT Tests
		)
endif()


INSTALL(FILES 
	"${PROJECT_BINARY_DIR}/TmatrixRandomConfigVersion.cmake"
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/TmatrixRandomConfig.cmake"
	DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT Libraries)
INSTALL(EXPORT TmatrixRandomTargets DESTINATION
	"${INSTALL_CMAKE_DIR}" COMPONENT Libraries)

