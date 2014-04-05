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
		"Build shapefile BOV import / export plugin" OFF)
	if (BUILD_PLUGIN_BOV)
		add_subdirectory (bov)
	endif()
	option (BUILD_PLUGIN_CERES_SOLVER
		"Build Google Ceres Solver plugin (optimization)" ON)
	if (BUILD_PLUGIN_CERES_SOLVER)
		add_subdirectory (ceres-solver)
	endif()
	option (BUILD_PLUGIN_HDF5
		"Build HDF5 IO plugin" ON)
	if (BUILD_PLUGIN_HDF5)
		add_subdirectory (hdf5)
	endif()
	option (BUILD_PLUGIN_IMAGEMAGICK
		"Build ImageMagick IO plugin" ON)
	if (BUILD_PLUGIN_IMAGEMAGICK)
		add_subdirectory (ImageMagick)
	endif()
	option (BUILD_PLUGIN_NETCDF
		"Build NETCDF IO plugin" ON)
	if (BUILD_PLUGIN_NETCDF)
		add_subdirectory (netcdf)
	endif()
	option (BUILD_PLUGIN_ROOT
		"Build ROOT plugin (io and plotting)" ON)
	if (BUILD_PLUGIN_ROOT)
		add_subdirectory (root)
	endif()
	option (BUILD_PLUGIN_SILO
		"Build SILO IO plugin" ON)
	if (BUILD_PLUGIN_SILO)
		add_subdirectory (silo)
	endif()
	option (BUILD_PLUGIN_TSV
		"Build TSV export plugin" ON)
	if (BUILD_PLUGIN_TSV)
		add_subdirectory(tsv-export)
	endif()
endif()

