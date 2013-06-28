# CMake Rules
set(cmake-other files_src.cmake)
source_group("CMake Rules" FILES ${cmake-other})

# rtmath_core
set(rtmath_core_error
	rtmath/error/debug.h
	rtmath/error/debug_mem.h
	rtmath/error/debug_mem_class.h
	rtmath/error/error.h
	src/debug.cpp
	src/debug_mem.cpp
	src/error.cpp
	)
source_group("Debugging" FILES ${rtmath_core_error})

set(rtmath_core_general
	rtmath/command.h
	rtmath/common_templates.h
	rtmath/config.h
	#rtmath/coords.h
	rtmath/defs.h
	rtmath/denseMatrix.h
	rtmath/density.h
	rtmath/depGraph.h
	rtmath/derivatives.h
	rtmath/interpolatable.h
	rtmath/macros.h
	rtmath/matrixop.h
	rtmath/phaseFunc.h
	rtmath/polynomial.h
	rtmath/quadrature.h
	rtmath/refract.h
	rtmath/splitSet.h
	rtmath/thermal.h
	rtmath/units.h
	rtmath/zeros.h
	src/command.cpp
	src/config.cpp
	#src/coords.cpp
	src/denseMatrix.cpp
	src/density.cpp
	src/depGraph.cpp
	#src/matrixop.cpp
	#src/os_functions.cpp
	src/phaseFunc.cpp
	src/polynomial.cpp
	src/quadrature.cpp
	src/refract.cpp
	src/splitSet.cpp
	src/thermal.cpp
	src/units.cpp
	src/zeros.cpp
	)
source_group("General" FILES ${rtmath_core_general})


set(rtmath_core_polynomials
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


set(rtmath_core_publicdomain
	rtmath/Public_Domain/MurmurHash3.h
	src/MurmurHash3.cpp
	)
source_group("Public Domain" FILES ${rtmath_core_publicdomain})


set(rtmath_core_serialization
	rtmath/Serialization/common_templates_serialization.h
	rtmath/Serialization/eigen_serialization.h
	#rtmath/Serialization/matrixop_serialization.h
	rtmath/Serialization/serialization_macros.h
	src/common_templates_serialization.cpp
	#src/eigen_serialization.cpp
	#src/matrixop_serialization.cpp
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
	rtlogo.png
	)
source_group("Resource Files" FILES ${resource-files})

set(rtmath_ddscat-files
	#rtmath/ddscat/cdf-ddscat.h
	rtmath/ddscat/ddavg.h
	rtmath/ddscat/ddLoader.h
	rtmath/ddscat/ddOutput.h
	rtmath/ddscat/ddOutputEnsemble.h
	rtmath/ddscat/ddOutputSingle.h
	rtmath/ddscat/ddpar.h
	rtmath/Serialization/ddpar_serialization.h
	rtmath/ddscat/ddparGenerator.h
	rtmath/Serialization/ddparGenerator_serialization.h
	rtmath/ddscat/ddscat.h
	rtmath/ddscat/ddScattMatrix.h
	rtmath/Serialization/ddScattMatrix_serialization.h
	rtmath/ddscat/ddVersions.h
	rtmath/ddscat/ddweights.h
	rtmath/ddscat/dielTabFile.h
	rtmath/ddscat/hulls.h
	rtmath/ddscat/mtab.h
	rtmath/ddscat/parids.h
	rtmath/ddscat/rotations.h
	rtmath/Serialization/rotations_serialization.h
	rtmath/ddscat/runScripts.h
	rtmath/ddscat/shapefile.h
	rtmath/Serialization/shapefile_serialization.h
	rtmath/ddscat/shapes.h
	rtmath/Serialization/shapes_serialization.h
	rtmath/ddscat/shapestats.h
	rtmath/Serialization/shapestats_serialization.h
	rtmath/ddscat/shapestatsRotated.h
	rtmath/Serialization/shapestatsRotated_serialization.h
	rtmath/ddscat/shapestatsviews.h
	rtmath/ddscat/tmData.h
	rtmath/Serialization/tmData_serialization.h
	#	src/ddLoader.cpp
	src/ddOutputSingle.cpp
	src/ddpar.cpp
	src/ddpar_serialization.cpp
	src/ddparGenerator.cpp
	src/ddparGenerator_serialization.cpp
	src/ddscat.cpp
	src/ddScattMatrix.cpp
	src/ddScattMatrix_serialization.cpp
	src/ddVersions.cpp
	src/ddweights.cpp
	src/dielTabFile.cpp
	src/hulls.cpp
	# src/mtab.cpp
	#	src/pclstuff.cpp
	src/rotations.cpp
	src/rotations_serialization.cpp
	src/runScripts.cpp
	src/shapefile.cpp
	src/shapefile_serialization.cpp
	src/shapes.cpp
	src/shapes_serialization.cpp
	src/shapestats.cpp
	src/shapestats_serialization.cpp
	src/shapestats_genstats.cpp
	src/shapestatsRotated.cpp
	src/shapestatsRotated_serialization.cpp
	src/shapes-generators.cpp
	src/tmData.cpp
	src/tmData_serialization.cpp
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
	)

# Final rules to provide with build targets
set (rtmath_core-files
	${cmake-other}
	${resource-files}
	${rtmath_core_error}
	${rtmath_core_general}
	${rtmath_core_polynomials}
	${rtmath_core_publicdomain}
	${rtmath_core_serialization}
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

