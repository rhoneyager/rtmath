# The slices library contains all of the compiled slice definitions

FILE( GLOB SLICE_FILES *.ice)

source_group("Slice Definitions" FILES ${SLICE_FILES})
#message("${SLICE_FILES}")
#get_filename_component( SLICE_FILES_NAMES ${SLICE_FILES} NAME)
#message("${SLICE_FILES_NAMES}")

#string( REGEX REPLACE "\\.ice" ".cpp" SLICE_CPP ${SLICE_FILES})
#string( REGEX REPLACE "\\.ice" ".h" SLICE_H ${SLICE_FILES})

foreach (filefull ${SLICE_FILES})
string( REGEX REPLACE "\\.ice" ".cpp" SLICE_CPP ${filefull})
string( REGEX REPLACE "\\.ice" ".h" SLICE_H ${filefull})

get_filename_component(filename ${SLICE_CPP} NAME)
get_filename_component(filenameh ${SLICE_H} NAME)

set (slice_cpp_files ${slice_cpp_files} ${filename} ${filenameh})
endforeach()

#message("${SLICE_CPP}")
#message("${SLICE_H}")
# Execute the slice2cpp command on these files to generate cpp and h files.

add_custom_command(OUTPUT ${slice_cpp_files} COMMAND ${SLICE2CPP} -I${ICE_SLICE_DIR} ${SLICE_FILES})
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/config.h"
	"${CMAKE_CURRENT_BINARY_DIR}/config.h"
	)

set(srcs
	${slice_cpp_files} 
	"${CMAKE_CURRENT_BINARY_DIR}/config.h" 
	${SLICE_FILES}
	)

set(COMPONENTNAME oneellip_slices)
set(COMPONENTDESC "Network interface definitions")
#configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../src/lib.rc.in"
	#	"${CMAKE_CURRENT_BINARY_DIR}/libslices.rc" @ONLY)

#if(WIN32 AND NOT CYGWIN)
	#set (srcs ${srcs} "${CMAKE_CURRENT_BINARY_DIR}/libslices.rc" )
	#endif()

include(addIceUniform)
addIceUniform(freeze ice icebox icedb icegrid icessl icepatch2 icestorm iceutil slice icexml)
include(addlib)

include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${ICE_INCLUDE_DIR})
add_library(oneellip_slices SHARED ${srcs} )
addlib(oneellip_slices SHARED)
storebin(oneellip_slices) # Put binaries in the usual directory
set(slices-libs ${ICE_LIBRARIES})
target_link_libraries(oneellip_slices ${slices-libs})

