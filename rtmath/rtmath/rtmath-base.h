#pragma once
// This is the base header, common to all rtmath projects
// This is as basic as you can get (used for stuff like struct definitions
// for scattering phase func., debug system primatives, and similar stuff)

#include "enums.h"
#include "phaseFunc.h"
#include "polynomial.h"
#include "zeros.h"
#include "quadrature.h"
#include "matrixop.h"

// Macro definitions to speed things up
// (like a redefine of atof on windows)
#include "macros.h"
