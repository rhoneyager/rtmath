// rtmath.h
/* This is the main header for the library.
 * All programs which link to this library should
 * include this header.
 */

#pragma once

#define BOOST_FILESYSTEM_VERSION 3

#include "error/debug.h"
#include "error/error.h"
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
#include "lbl.h"
#include "atmos.h"
#include "config.h"
#include "command.h"
#include "mie/mie.h"
#include "rayleigh/rayleigh.h"
#include "ddscat.h"
#include "cdf-ddscat.h"

