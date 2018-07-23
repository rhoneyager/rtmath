# Build the apps
#include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
#find_package(Ryan_Debug REQUIRED)
#find_package(Ryan_Serialization REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(EIGEN3_INCLUDE_DIR)
#include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
#include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})

if (BUILD_PLUGINS)
	# TODO: Add to a separate repository
	option (BUILD_PLUGIN_BHMIE
		"Build Bohren and Huffman Mie plugin" ON)
	if (BUILD_PLUGIN_BHMIE)
		add_subdirectory(bhmie)
	endif()

	# Deprecated
	#option (BUILD_PLUGIN_BOV
	#	"Build shapefile BOV import / export plugin" OFF)
	#if (BUILD_PLUGIN_BOV)
	#	add_subdirectory (bov)
	#endif()

	# Never fully implemented
	#option (BUILD_PLUGIN_CERES_SOLVER
	#	"Build Google Ceres Solver plugin (optimization)" OFF)
	#if (BUILD_PLUGIN_CERES_SOLVER)
	#	add_subdirectory (ceres-solver)
	#endif()

	# Ubuntu: libflann-dev
	# TODO: Add FindFLANN.cmake
	option (BUILD_PLUGIN_FLANN
		"Build FLANN K-d tree plugin" ON)
	if (BUILD_PLUGIN_FLANN)
		add_subdirectory (flann)
	endif()

	# Ubuntu: libhdf5-dev
	option (BUILD_PLUGIN_HDF5
		"Build HDF5 IO plugin" ON)
	if (BUILD_PLUGIN_HDF5)
		add_subdirectory (hdf5)
	endif()

	option (BUILD_PLUGIN_IMAGEMAGICK
		"Build ImageMagick IO plugin" ON)
	# Ubuntu libmagick++-dev
	if (BUILD_PLUGIN_IMAGEMAGICK)
		add_subdirectory (ImageMagick)
	endif()

	# My own Mie code
	option (BUILD_PLUGIN_MIE
		"Build Mie plugin" ON)
	if (BUILD_PLUGIN_MIE)
		add_subdirectory(mie)
	endif()
	# TODO: Complaind about missing Find___.cmake if the base packages are not installed.
	# Instead, detect if they are missing, then request that the builder install
	# these packages.
	
	# TODO: Needs Findnetcdf.cmake
	# Ubuntu libnetcdf-dev libnetcdf-c++4-dev
	option (BUILD_PLUGIN_NETCDF
		"Build NETCDF IO plugin" ON)
	if (BUILD_PLUGIN_NETCDF)
		add_subdirectory (netcdf)
	endif()

	# My SSRGA and others.
	# TODO: Add a separate plugin for the newest SSRGA, and place in a related repository.
	option (BUILD_PLUGIN_NONDDA_MISC
		"Build miscellaneous non-DDA plugins (Rayleigh, Rayleigh-Gans,
		SSRGA)" ON)
		if (BUILD_PLUGIN_NONDDA_MISC)
		add_subdirectory(nondda-misc)
	endif()

	# Ubuntu: libpq-dev
	option (BUILD_PLUGIN_PSQL
		"Build PostgreSQL IO plugin" ON)
	if (BUILD_PLUGIN_PSQL)
		add_subdirectory (psql)
	endif()

	# Ubuntu libqhull-dev
	# Unnecessary for now
	option (BUILD_PLUGIN_QHULL
		"Build quickhull plugin" OFF)
	if (BUILD_PLUGIN_QHULL)
		add_subdirectory(qhull)
	endif()

	# Deprecated
	#option (BUILD_PLUGIN_ROOT
	#	"Build ROOT plugin (io and plotting)" OFF)
	#if (BUILD_PLUGIN_ROOT)
	#	add_subdirectory (root)
	#endif()

	# Will be in a related repository
	#option (BUILD_PLUGIN_SCATDB_RYAN
	#	"Scatdb_Ryan plugin provides database I/O" ON)
	#if (BUILD_PLUGIN_SCATDB_RYAN)
	#	add_subdirectory (scatdb_ryan)
	#endif()

	# TODO: Findsilo.cmake. I have to provide the CMake addendum for building Silo.
	# Ubuntu libsilo-dev
	option (BUILD_PLUGIN_SILO
		"Build SILO IO plugin" OFF)
	if (BUILD_PLUGIN_SILO)
		add_subdirectory (silo)
	endif()

	#option (BUILD_PLUGIN_TMATRIX
	#	"Build Tmatrix plugin" ON)
	#if (BUILD_PLUGIN_TMATRIX)
	#	add_subdirectory(tmatrix)
	#endif()
	option (BUILD_PLUGIN_TMATRIX_RANDOM
		"Build Tmatrix random orientation plugin" ON)
	if (BUILD_PLUGIN_TMATRIX_RANDOM)
		add_subdirectory(tmatrix-random)
	endif()

	option (BUILD_PLUGIN_TSV
		"Build TSV export plugin" ON)
	if (BUILD_PLUGIN_TSV)
		add_subdirectory(tsv-export)
	endif()

	# Ubuntu: libudunits2-dev
	option (BUILD_PLUGIN_UDUNITS
		"Build udunits plugin. Supplements unit conversions." ON)
	if (BUILD_PLUGIN_UDUNITS)
		add_subdirectory(udunits)
	endif()

	# Ubuntu: voro++-dev
	option (BUILD_PLUGIN_VORO
		"Build VORO++ 3d plugin. Used in Voronoi diagram." ON)
	if (BUILD_PLUGIN_VORO)
		add_subdirectory(voro)
	endif()

	# Need to build by hand
	option (BUILD_PLUGIN_VORO2D
		"Build VORO++ 2d plugin. Used in stats." OFF)
	if (BUILD_PLUGIN_VORO2D)
		add_subdirectory(voro2d)
	endif()

	# Ubuntu: libvtk6-dev
	option (BUILD_PLUGIN_VTK
		"Build VTK plugin. Used in hull calculation, image processing and visualization." ON)
	if (BUILD_PLUGIN_VTK)
		add_subdirectory(vtk)
	endif()
endif()

