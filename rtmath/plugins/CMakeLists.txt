# Build the apps
find_package(Boost COMPONENTS program_options 
	serialization iostreams filesystem system unit_test_framework regex REQUIRED)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
find_package(Ryan_Debug REQUIRED)
find_package(Ryan_Serialization REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})

option (BUILD_PLUGINS
	"Master switch to enable / disable building of plugins" ON)
if (BUILD_PLUGINS)
	option (BUILD_PLUGIN_BOV
		"Build shapefile BOV import / export plugin" ON)
	if (BUILD_PLUGIN_BOV)
		add_subdirectory (bov)
	endif()
endif()

