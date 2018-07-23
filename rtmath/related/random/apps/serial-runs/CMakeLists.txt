include (common)
find_package(Ryan_Debug REQUIRED)
find_package(Ryan_Serialization REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})
#find_package(Boost COMPONENTS program_options 
#	serialization iostreams filesystem system REQUIRED)

add_executable (tmatrix-serial-runs
	tmatrix-serial-runs.cpp
	)
set_target_properties( tmatrix-serial-runs
	PROPERTIES FOLDER "Apps" )
target_link_libraries( tmatrix-serial-runs
	${liblist}
	tmatrix_cpp tmatrix_fortran)
SET_TARGET_PROPERTIES( tmatrix-serial-runs PROPERTIES RELEASE_POSTFIX _Release )
SET_TARGET_PROPERTIES( tmatrix-serial-runs PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel )
SET_TARGET_PROPERTIES( tmatrix-serial-runs PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo )
SET_TARGET_PROPERTIES( tmatrix-serial-runs PROPERTIES DEBUG_POSTFIX _Debug)
IF(DEFINED COMMON_CFLAGS )
	set_target_properties(tmatrix-serial-runs PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()
