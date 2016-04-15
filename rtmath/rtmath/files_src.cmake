# CMake Rules
set(cmake-other files_src.cmake)
source_group("CMake Rules" FILES ${cmake-other})

# rtmath_core
set(rtmath_core_error
	rtmath/error/debug.h
	#rtmath/error/debug_mem.h
	#rtmath/error/debug_mem_class.h
	rtmath/error/error.h
	src/debug.cpp
	#src/debug_mem.cpp
	#src/error.cpp
	src/info.cpp
	rtmath/info.h
	)
source_group("Debugging" FILES ${rtmath_core_error})

set(rtmath_core_config
	#rtmath/command.h
	rtmath/config.h
	rtmath/registry.h
	rtmath/plugin.h
	#src/command.cpp
	src/config.cpp
	src/registry.cpp
	)
source_group("Configuration" FILES ${rtmath_core_config})

set(rtmath_core_general
	rtmath/common_templates.h
	rtmath/conversions/convertLength.h
	rtmath/conversions/unitOptions.h
	#rtmath/coords.h
	rtmath/defs.h
	rtmath/denseMatrix.h
	rtmath/density/density.h
	rtmath/density/densityImpl.h
	rtmath/depGraph.h
	rtmath/derivatives.h
	#rtmath/hash.h
	rtmath/interpolatable.h
	rtmath/macros.h
	#rtmath/matrixop.h
	rtmath/phaseFunc.h
	rtmath/psd.h
	rtmath/quadrature.h
	rtmath/refract.h
	#rtmath/splitSet.h
	rtmath/thermal.h
	rtmath/units.h
	rtmath/zeros.h
	#src/coords.cpp
	src/denseMatrix.cpp
	src/density.cpp
	src/depGraph.cpp
	#src/hash.cpp
	src/macros.cpp
	#src/matrixop.cpp
	#src/os_functions.cpp
	src/phaseFunc.cpp
	src/psd.cpp
	src/quadrature.cpp
	src/refract.cpp
	#src/splitSet.cpp
	src/thermal.cpp
	src/units.cpp
	src/zeros.cpp
	)
source_group("General" FILES ${rtmath_core_general})


set(rtmath_core_polynomials
	rtmath/polynomial.h
	src/polynomial.cpp
	rtmath/polynomials/chebyshev.h
	rtmath/polynomials/hermite.h
	rtmath/polynomials/laguerre.h
	rtmath/polynomials/legendre.h
	rtmath/polynomials/recursivePolynomial.h
	src/chebyshev.cpp
	src/hermite.cpp
	src/laguerre.cpp
	src/legendre.cpp
	#src/recursivePolynomial.cpp
	)
source_group("Polynomials" FILES ${rtmath_core_polynomials})


#set(rtmath_core_publicdomain
#	rtmath/Public_Domain/MurmurHash3.h
#	src/MurmurHash3.cpp
#	)
#source_group("Public Domain" FILES ${rtmath_core_publicdomain})


set(rtmath_core_serialization
	#rtmath/io.h
	#rtmath/Serialization/eigen_serialization.h
	#rtmath/Serialization/matrixop_serialization.h
	rtmath/Serialization/serialization_macros.h
	rtmath/Serialization/Serialization.h
	#src/common_templates_serialization.cpp
	#src/io.cpp
	#src/eigen_serialization.cpp
	#src/matrixop_serialization.cpp
	#src/serialization.cpp
	)
source_group("Serialization" FILES ${rtmath_core_serialization})




# Resource Files
set (resource-files
	app.ico
	app.rc
	getRev.csh
	prebuild.bat
	prebuild.csh
	Readme.txt
	logo.png
	)
source_group("Resource Files" FILES ${resource-files})



set(rtmath_voronoi-files
	rtmath/Voronoi/CachedVoronoi.h
	rtmath/Voronoi/Voronoi.h
	rtmath/ddscat/hulls.h
	rtmath/ddscat/points.h
	src/CachedVoronoi.cpp
	src/hulls.cpp
	src/points.cpp
	src/Stdafx-voronoi.cpp
	src/Stdafx-voronoi.h
	src/Voronoi.cpp
	#src/Voronoi_Serialization.cpp
	)

set(rtmath_ddscat_base-files
	rtmath/ddscat/ddpar.h
	rtmath/ddscat/parids.h
	src/ddpar.cpp
	src/ddpar-keys.cpp
	src/ddpar-parsers.cpp
	#src/ddpar_serialization.cpp
	rtmath/ddscat/rotations.h
	src/rotations.cpp
	#src/rotations_serialization.cpp
	rtmath/ddscat/dielTabFile.h
	src/dielTabFile.cpp
	rtmath/ddscat/ddVersions.h
	src/ddVersions.cpp
	
	src/Stdafx-ddscat_base.cpp
	src/Stdafx-ddscat_base.h
	)

set(rtmath_ddscat-shape-files
	rtmath/ddscat/shapefile.h
	rtmath/ddscat/shapefile_supplemental.h
	#rtmath/ddscat/shapes.h
	rtmath/ddscat/shapestats.h
	src/shapestats_private.h
	rtmath/ddscat/shapestatsRotated.h
	rtmath/ddscat/shapestatsviews.h
	src/shapefile.cpp
	src/shapefile_convolute.cpp
	src/shapefile_parsers.cpp
	src/shapefile_query.cpp
	src/shapefile_supplemental.cpp
	#src/shapefile_serialization.cpp
	#src/shapes.cpp
	#src/shapes_serialization.cpp
	src/shapestats.cpp
	src/shapestats_basic.cpp
	#src/shapestats_serialization.cpp
	src/shapestatsRotated.cpp
	#src/shapestatsRotated_serialization.cpp
	#src/shapestats_genstats.cpp
	#src/shapes-generators.cpp
	)
source_group("Shapes" FILES ${rtmath_ddscat-shape-files})
set(rtmath_ddscat-ddoutput-files
	rtmath/ddscat/ddOutput.h
	#rtmath/ddscat/ddOutputGenerator.h
	#rtmath/ddscat/ddOutputSingle.h
	rtmath/ddscat/ddOriData.h
	rtmath/ddscat/ddScattMatrix.h
	src/ddOriData.cpp
	src/ddOriDataParsers.cpp
	src/ddOriDataParsers.h
	src/ddOutput.cpp
	src/ddOutput_query.cpp
	#src/ddOutput_serialization.cpp
	#src/ddOutputGenerator.cpp
	#src/ddOutputGenerator_serialization.cpp
	#src/ddOutputSingle.cpp
	#src/ddOutputSingle_b.cpp
	#src/ddOutputSingle_c.cpp
	#src/ddOutputSingle_keys.cpp
	#src/ddOutputSingle_keys_b.cpp
	src/ddScattMatrix.cpp
	#src/ddScattMatrix_serialization.cpp
	)
source_group("DDSCAT Output" FILES ${rtmath_ddscat-ddoutput-files})

set(rtmath_ddscat-files
	#rtmath/ddscat/cdf-ddscat.h
	rtmath/ddscat/ddavg.h
	rtmath/ddscat/ddLoader.h
	#rtmath/ddscat/ddparGenerator.h
	rtmath/ddscat/ddRunSet.h
	rtmath/ddscat/ddscat.h
	rtmath/ddscat/ddUtil.h
	rtmath/ddscat/ddweights.h
	rtmath/ddscat/mtab.h
	#rtmath/ddscat/runScripts.h
	rtmath/ddscat/tmData.h
	src/ddavg.cpp
	# src/ddLoader.cpp
	#src/ddparGenerator.cpp
	#src/ddparGenerator_serialization.cpp
	src/ddRunSet.cpp
	#src/ddscat.cpp
	src/ddUtil.cpp
	src/ddweights.cpp	
	# src/mtab.cpp
	# src/pclstuff.cpp
	#src/runScripts.cpp
	#src/tmData.cpp
	#src/tmData_serialization.cpp
	src/Stdafx-ddscat.cpp
	src/Stdafx-ddscat.h
	${rtmath_ddscat-shape-files}
	${rtmath_ddscat-ddoutput-files}
	rtmath/rga/rga.h
	src/rga.cpp
	)
#source_group("Resource Files" FILES ${resource-files})


set(rtmath_mie-files
	rtmath/mie/mie.h
	rtmath/mie/mie-abNCalc.h
	rtmath/mie/mie-AnCalc.h
	rtmath/mie/mie-piNCalc.h
	rtmath/mie/mie-Qcalc.h
	rtmath/mie/mie-Scalc.h
	rtmath/mie/mie-tauNCalc.h
	rtmath/mie/mie-wnCalc.h
	src/mie.cpp
	src/mie-abNCalc.cpp
	src/mie-AnCalc.cpp
	src/mie-piNCalc.cpp
	src/mie-Qcalc.cpp
	src/mie-Scalc.cpp
	src/mie-serialization.cpp
	src/mie-tauNCalc.cpp
	src/mie-wnCalc.cpp
	src/Stdafx-mie.cpp
	src/Stdafx-mie.h
	)

set(rtmath_rt-files
	src/Stdafx-rt.cpp
	src/Stdafx-rt.h
	rtmath/atmos/absorb.h
	#src/absorb.cpp
	#src/absorber.cpp
	rtmath/atmos/atmos.h
	#src/atmos.cpp
	rtmath/atmos/atmoslayer.h
	#src/atmoslayer.cpp
	rtmath/atmos/hitran.h
	#src/hitran.cpp
	)

set(rtmath_da-files
	)

set(rtmath_mc-files
	)

set(rtmath_data-files
	src/Stdafx-data.cpp
	src/Stdafx-data.h
	src/arm_info.cpp
	rtmath/data/arm_info.h
	src/arm_scanning_radar_sacr.cpp
	rtmath/data/arm_scanning_radar_sacr.h
	)

set(rtmath_images-files
	src/Stdafx-images.cpp
	src/Stdafx-images.h
	src/imageSet.cpp
	src/image.cpp
	rtmath/images/image.h
	rtmath/images/imageSet.h
	)

# Final rules to provide with build targets
set (rtmath_core-files
	${cmake-other}
	${resource-files}
	${rtmath_core_config}
	${rtmath_core_error}
	${rtmath_core_general}
	${rtmath_core_polynomials}
	${rtmath_core_publicdomain}
	${rtmath_core_serialization}
	src/Stdafx-core.cpp
	src/Stdafx-core.h
	)

set (rtmath_core_test-files
	tests/test.cpp
	#tests/test_matrixop.cpp
	tests/globals.cpp
	tests/globals.h
	tests/test_quadrature.cpp
	tests/test_polynomial.cpp
	tests/test_units.cpp
	tests/test_debug.cpp
	tests/test_recPolys.cpp
	# TODO: reenable other tests
	)

set (rtmath_mie_test-files
	tests/test.cpp
	tests/globals.cpp
	tests/globals.h
	tests/mie-twn.cpp
	)

set (rtmath_ddscat_test-files
	tests/test.cpp
	tests/globals.cpp
	tests/globals.h
	tests/test_ddpar.cpp
	#tests/test_ddparGenerator.cpp
	tests/test_ddOutputSingle.cpp
	#tests/test_ddScattMatrix.cpp
	tests/test_shapefile.cpp
	#tests/test_shapestats.cpp
	# Add rotations, shapestats_scaled, hulls, 
	# ddversions, ddweights, ensemble functions, 
	# tmdata, dieltabfile
	)

