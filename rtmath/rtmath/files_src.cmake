# CMake Rules
set(cmake-other files_src.cmake)
source_group("CMake Rules" FILES ${cmake-other})

# Atmos
set (atmos-src
	src/absorb.cpp
	src/absorber.cpp
	src/atmos.cpp
	src/atmoslayer.cpp
	)
set (atmos-hdr
	rtmath/absorb.h
	rtmath/atmos.h
	rtmath/atmoslayer.h
	rtmath/lbl.h
	)
source_group("Source Files\\Atmos" FILES ${atmos-src})
source_group("Header Files\\Atmos" FILES ${atmos-hdr})

# da
set (da-src
	src/daDiagonalMatrix.cpp
	src/daInitLayer.cpp
	src/daLayer.cpp
	src/damatrix.cpp
	src/damatrix_quad.cpp
	src/daPf.cpp
	src/daStatic.cpp
	)
set (da-hdr
	rtmath/da/daDiagonalMatrix.h
	rtmath/da/daInitLayer.h
	rtmath/da/daLayer.h
	rtmath/da/damatrix.h
	rtmath/da/damatrix_override.h
	rtmath/da/damatrix_quad.h
	rtmath/da/daPf.h
	rtmath/da/daStatic.h
	)
source_group("Source Files\\da" FILES ${da-src})
source_group("Header Files\\da" FILES ${da-hdr})

# ddscat
set (ddscat-src
	src/ddLoader.cpp
	src/ddOutput.cpp
	src/ddOutputEnsemble.cpp
	src/ddOutputSingle.cpp
	src/ddpar.cpp
	src/ddparGenerator.cpp
	src/ddscat.cpp
	src/ddScattMatrix.cpp
	src/ddweights.cpp
	src/mtab.cpp
	src/shapefile.cpp
	src/shapes.cpp
	)
set (ddscat-hdr
	rtmath/ddscat/cdf-ddscat.h
	rtmath/ddscat/ddLoader.h
	rtmath/ddscat/ddOutput.h
	rtmath/ddscat/ddOutputEnsemble.h
	rtmath/ddscat/ddOutputSingle.h
	rtmath/ddscat/ddpar.h
	rtmath/ddscat/ddparGenerator.h
	rtmath/ddscat/ddscat.h
	rtmath/ddscat/ddScattMatrix.h
	rtmath/ddscat/ddweights.h
	rtmath/ddscat/mtab.h
	rtmath/ddscat/parids.h
	rtmath/ddscat/shapefile.h
	rtmath/ddscat/shapes.h
	)
source_group("Source Files\\ddscat" FILES ${ddscat-src})
source_group("Header Files\\ddscat" FILES ${ddscat-hdr})

# error
set (error-src
	src/debug.cpp
	src/debug_mem.cpp
	src/error.cpp
	)
set (error-hdr
	rtmath/error/debug.h
	rtmath/error/debug_mem.h
	rtmath/error/debug_mem_class.h
	rtmath/error/debug_subversion.h.template
	rtmath/error/error.h
	)
source_group("Source Files\\error" FILES ${error-src})
source_group("Header Files\\error" FILES ${error-hdr})

# General
set (general-src
	src/command.cpp
	src/config.cpp
	src/coords.cpp
	src/depGraph.cpp
	src/emd.cpp
	src/interpolatable.cpp
	src/matrixop.cpp
	src/matrixop-ipp.cpp
	src/os_functions.cpp
	src/pfFixed.cpp
	src/phaseFunc.cpp
	src/polynomial.cpp
	src/quadrature.cpp
	src/refract.cpp
	src/ROOT_functions.cpp
	src/rtmath.cpp
	src/Stdafx.cpp
	src/surfaces.cpp
	src/thermal.cpp
	src/units.cpp
	src/zeros.cpp
	)
set (general-hdr
	rtmath/command.h
	rtmath/common_templates.h
	rtmath/config.h
	rtmath/coords.h
	rtmath/defs.h
	rtmath/depGraph.h
	rtmath/emd.h
	rtmath/enums.h
	rtmath/interpolatable.h
	rtmath/macros.h
	rtmath/mapid.h
	rtmath/matrixop.h
	rtmath/parsers.h
	rtmath/pfFixed.h
	rtmath/phaseFunc.h
	rtmath/polynomial.h
	rtmath/quadrature.h
	rtmath/refract.h
	rtmath/resource.h
	rtmath/ROOT_functions.h
	rtmath/ROOTlink.h
	rtmath/rtmath.h
	rtmath/Stdafx.h
	rtmath/units.h
	rtmath/zeros.h
	)
source_group("Source Files\\General" FILES ${general-src})
source_group("Header Files\\General" FILES ${general-hdr})

# gridded
set (gridded-src
	src/gridCDF.cpp
	src/gridded.cpp
	src/gridStatic.cpp
	)
set (gridded-hdr
	rtmath/gridded/gridCDF.h
	rtmath/gridded/gridCoords.h
	rtmath/gridded/gridded.h
	rtmath/gridded/gridStatic.h
	)
source_group("Source Files\\gridded" FILES ${gridded-src})
source_group("Header Files\\gridded" FILES ${gridded-hdr})

# mie
set (mie-src
	src/ddOutputMie.cpp
	src/mie-abNCalc.cpp
	src/mie-AnCalc.cpp
	src/mie-phaseFunc.cpp
	src/mie-piNCalc.cpp
	src/mie-Qcalc.cpp
	src/mie-Scalc.cpp
	src/mieScattMatrix.cpp
	src/mie-tauNCalc.cpp
	src/mie-wnCalc.cpp
	)
set (mie-hdr
	rtmath/mie/ddOutputMie.h
	rtmath/mie/mie.h
	rtmath/mie/mie-abNCalc.h
	rtmath/mie/mie-AnCalc.h
	rtmath/mie/mie-phaseFunc.h
	rtmath/mie/mie-piNCalc.h
	rtmath/mie/mie-Qcalc.h
	rtmath/mie/mie-Scalc.h
	rtmath/mie/mieScattMatrix.h
	rtmath/mie/mie-tauNCalc.h
	rtmath/mie/mie-wnCalc.h
	)
source_group("Source Files\\mie" FILES ${mie-src})
source_group("Header Files\\mie" FILES ${mie-hdr})

# netcdf
set (netcdf-src
	src/ncContainer.cpp
	)
set (netcdf-hdr
	rtmath/netcdf/ncContainer.h
	rtmath/netcdf/ncDim.h
	rtmath/netcdf/ncVar.h
	)
source_group("Source Files\\netcdf" FILES ${netcdf-src})
source_group("Header Files\\netcdf" FILES ${netcdf-hdr})

# Polynomials
set (polynomials-src
	src/chebyshev.cpp
	src/hermite.cpp
	src/laguerre.cpp
	src/legendre.cpp
	src/recursivePolynomial.cpp
	)
set (polynomials-hdr
	rtmath/polynomials/chebyshev.h
	rtmath/polynomials/hermite.h
	rtmath/polynomials/laguerre.h
	rtmath/polynomials/legendre.h
	rtmath/polynomials/recursivePolynomial.h
	)
source_group("Source Files\\Polynomials" FILES ${polynomials-src})
source_group("Header Files\\Polynomials" FILES ${polynomials-hdr})

# Public Domain
set (publicdomain-src
	src/bhmie.cpp
	src/MurmurHash3.cpp
	)
set (publicdomain-hdr
	rtmath/Public_Domain/bhmie.h
	rtmath/Public_Domain/MurmurHash3.h
	)
source_group("Source Files\\Public Domain" FILES ${publicdomain-src})
source_group("Header Files\\Public Domain" FILES ${publicdomain-hdr})

# rayleigh
set (rayleigh-src
	src/Qcalc-Rayleigh.cpp
	src/rayleighPhaseFunc.cpp
	src/Scalc-Rayleigh.cpp
	)
set (rayleigh-hdr
	rtmath/rayleigh/Qcalc-Rayleigh.h
	rtmath/rayleigh/rayleigh.h
	rtmath/rayleigh/rayleighPhaseFunc.h
	rtmath/rayleigh/Scalc-Rayleigh.h
	)
source_group("Source Files\\rayleigh" FILES ${rayleigh-src})
source_group("Header Files\\rayleigh" FILES ${rayleigh-hdr})

# /Other

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



# Final rules to provide with build targets
set (srcfiles
	${cmake-other}
	${resource-files}
	${atmos-src}			${atmos-hdr}
	${da-src}				${da-hdr}
	${ddscat-src}			${ddscat-hdr}
	${error-src}			${error-hdr}
	${general-src}			${general-hdr}
	${gridded-src}			${gridded-hdr}
	${mie-src}				${mie-hdr}
	${netcdf-src}			${netcdf-hdr}
	${polynomials-src}		${polynomials-hdr}
	${publicdomain-src}		${publicdomain-hdr}
	${rayleigh-src}			${rayleigh-hdr}
	)

# And we're done

