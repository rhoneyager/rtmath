# This is a scratch project, implementing Ice into 
# the tmatrix code. The slice definitions and default 
# implementations will eventually become part of the 
# tmatrix-cpp project, and the client components will 
# remain here.

# TODO: add windows .rc file

include (common)
if (NOT BUILD_FORTRAN)
	# Attempt to import a prebuilt tmatrix-fortran library here
	# This is primarily useful for Windows non-g++, non-intel builds.
	find_package(tmatrixFortran REQUIRED)
endif()

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (tmatrix-ice-client
	main.cpp
	#main.h
	)

if (DEFINED ICE_INCLUDE_DIR) # Excepting windows
		include_directories (${ICE_INCLUDE_DIR})
endif()
include_directories (${CMAKE_BINARY_DIR})

SET_TARGET_PROPERTIES( tmatrix-ice-client PROPERTIES FOLDER "Apps")
SET_TARGET_PROPERTIES( tmatrix-ice-client PROPERTIES RELEASE_POSTFIX _Release )
SET_TARGET_PROPERTIES( tmatrix-ice-client PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel )
SET_TARGET_PROPERTIES( tmatrix-ice-client PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo )
SET_TARGET_PROPERTIES( tmatrix-ice-client PROPERTIES DEBUG_POSTFIX _Debug)
IF(DEFINED COMMON_CFLAGS )
	set_target_properties(tmatrix-ice-client PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()

target_link_libraries(tmatrix-ice-client
	tmatrix-cpp
	tmatrix-fortran
	Ryan_Debug
	Ryan_Serialization
	${liblist}
	)


