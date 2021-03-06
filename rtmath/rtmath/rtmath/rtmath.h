// rtmath.h
/* This is the main header for the library.
 * All programs which link to this library should
 * include this header.
 */

#pragma once

//#pragma warning( push )
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4503 ) // decorated name length exceeded. with boost bimap mpl

// Needed for compatability with older boost versions
#define BOOST_FILESYSTEM_VERSION 3

// Not an includive list, as several of these headers load other headers
#include "error/error.h"
#include "error/debug.h"
#include "enums.h"
#include "matrixop.h"
#include "polynomial.h"
#include "polynomials/recursivePolynomial.h"
#include "zeros.h"
#include "quadrature.h"
#include "Public_Domain/MurmurHash3.h"
#include "phaseFunc.h"
#include "macros.h"
#include "da/damatrix.h"
#include "da/daInitLayer.h"
#include "da/daLayer.h"
#include "da/daDiagonalMatrix.h"
#include "da/damatrix_override.h"
#include "da/damatrix_quad.h"
#include "da/daPf.h"
#include "da/daStatic.h"
//#include "lbl.h"
#include "atmos.h"
#include "absorb.h"
#include "atmoslayer.h"
#include "config.h"
#include "command.h"
#include "mie/mie.h"
#include "rayleigh/rayleigh.h"
#include "ddscat/ddscat.h"
#include "ddscat/cdf-ddscat.h"
#include "ddscat/shapefile.h"
#include "ddscat/shapestats.h"
#include "ddscat/hulls.h"
#include "ddscat/ddpar.h"
#include "ddscat/mtab.h"
#include "ddscat/shapes.h"
#include "ddscat/ddparGenerator.h"
#include "refract.h"
#include "units.h"
#include "ddscat/ddLoader.h"
#include "gridded/gridded.h"
#include "gridded/gridCoords.h"
#include "gridded/gridStatic.h"
#include "gridded/gridCDF.h"
#include "serialization.h"

//#include "ROOT_functions.h"

// Link with ROOT on MSVC
//#include "ROOTlink.h"

//#pragma warning( pop )
