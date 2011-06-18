// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <set>
#include <cstdlib>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <time.h>
#include <exception>
#include <new>

#ifdef _OPENMP
#include <omp.h>
#endif

//#ifdef HEAP_CHECK
// The heap-checking overridess to new and delete, to find bugs!!!
#include "debug_mem.h"
//#endif

#include "debug.h"
//#include "error.h"
//#include "damatrix.h"
