include (common)
include (commonRtmathApps)

find_package(rycurses REQUIRED)
add_executable (rtmath-ddscat-output-ingest-ncurses ddscat-output-ingest-ncurses.cpp)
target_link_libraries(rtmath-ddscat-output-ingest-ncurses ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat rycurses)
#include_directories(${RYCURSES_INCLUDE_DIRS})
addapp(rtmath-ddscat-output-ingest-ncurses Analysis)

if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-ddscat-output-ingest-ncurses rtmath_plugin_hdf5)
endif()

