include (common)
include (commonRtmathApps)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
find_package(Ice REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(protobuf)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS tmm.proto)

# Definitions of ICE communications layer
add_subdirectory (slices)

include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR}/slices)

include_directories(${ICE_INCLUDE_DIR})

#add_subdirectory (protobuf)
add_executable (rtmath-oneellip-parallel
	app.cpp
	app.h
	comm.cpp
	comm.h
	dispatcher.cpp
	dispatcher.h
	main.cpp
	main.h
	worker.cpp
	worker.h
	)
target_link_libraries( rtmath-oneellip-parallel 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	Qt5::Core Qt5::Network Qt5::Widgets
	oneellip_slices
	#oneellip_proto
	)
addapp(rtmath-oneellip-parallel Analysis)

if (BUILD_PLUGIN_BHMIE)
	add_dependencies ( rtmath-oneellip-parallel rtmath_plugin_bhmie)
endif()
if (BUILD_PLUGIN_MIE)
	add_dependencies ( rtmath-oneellip-parallel rtmath_plugin_mie)
endif()
if (BUILD_PLUGIN_NONDDA_MISC)
	add_dependencies ( rtmath-oneellip-parallel rtmath_plugin_nondda_misc)
endif()
if (BUILD_PLUGIN_TMATRIX)
	add_dependencies ( rtmath-oneellip-parallel rtmath_plugin_tmatrix)
endif()

