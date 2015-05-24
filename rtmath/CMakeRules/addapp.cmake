# CMake script for the very repetitive structure of 
# rtmath-apps CMakeLists.txt files.
include(signing)
# Take the variable {appname}, and link libraries,
# set properties and create an INSTALL target
macro(addapp appname foldername)
	#message("${foldername} ${appname}")
	set_target_properties( ${appname} PROPERTIES FOLDER "Apps/${foldername}")
	INSTALL(TARGETS ${appname} RUNTIME DESTINATION bin/bin@configappend@ COMPONENT Applications)
	
	#if (NOT ${appname}_INCLUDE_DIRS)
	#set(${appname}_LIBRARIES "")
	#set(${appname}_INCLUDE_DIRS "")
	#endif()

#add_package_dependency(${appname} DEPENDS_ON rtmathDummy)

#target_link_libraries (${appname} ${${appname}_LIBRARIES})
#IF(DEFINED COMMON_LIBS )
#	target_link_libraries (${appname} ${COMMON_LIBS})
#ENDIF()
#IF(DEFINED rtmath_INCLUDE_DIRS)
	# Other apps use this function. Only include rtmath if desired.
	#	target_link_libraries (${appname} rtmath tmatrix-cpp tmatrix-serialization tmatrix-fortran)
#ENDIF()
	include_directories(${CMAKE_CURRENT_BINARY_DIR})
	#	include_directories(${${appname}_INCLUDE_DIRS})


	IF(DEFINED COMMON_CFLAGS) 
		set_target_properties(${appname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	ENDIF()

	#IF(RTMATH_OLD_QT)
	#	add_definitions(-DRTMATH_OLD_QT)
	#ENDIF()

	storebin(${appname})
	delayedsigning( ${appname} )
endmacro(addapp appname)

macro(add_header_files srcs)
  if( hds )
    set_source_files_properties(${hds} PROPERTIES HEADER_FILE_ONLY ON)
    list(APPEND ${srcs} ${hds})
  endif()
endmacro(add_header_files srcs)

macro(addbasicapp appname)
	set(${appname}_LIBRARIES "")
	set(${appname}_INCLUDE_DIRS "")

	target_link_libraries (${appname} ${${appname}_LIBRARIES})
IF(DEFINED COMMON_LIBS )
	target_link_libraries (${appname} ${COMMON_LIBS})
ENDIF()
	include_directories(${CMAKE_CURRENT_BINARY_DIR})
	include_directories(${${appname}_INCLUDE_DIRS})


	IF(DEFINED COMMON_CFLAGS) 
		set_target_properties(${appname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	ENDIF()

	INSTALL(TARGETS ${appname} RUNTIME DESTINATION bin COMPONENT Applications)
	set_target_properties(${appname} PROPERTIES FOLDER "Apps")
	delayedsigning( ${appname} )
endmacro(addbasicapp appname)


