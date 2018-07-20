include(signing)
include_directories(${CMAKE_SOURCE_DIR}/ryan-debug/)

macro(addapp appname foldername)
	set_target_properties( ${appname} PROPERTIES FOLDER "Apps/${foldername}")
	INSTALL(TARGETS ${appname} RUNTIME DESTINATION bin/bin${configappend} COMPONENT Applications)
	include_directories(${CMAKE_CURRENT_BINARY_DIR})
	IF(DEFINED COMMON_CFLAGS) 
		set_target_properties(${appname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	ENDIF()

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


