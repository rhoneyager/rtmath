// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once

// Define filesystem version 3 for compile issues with older boost
#define BOOST_FILESYSTEM_VERSION 3

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <boost/filesystem.hpp>
#include <set>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <complex>
#include <fstream>
#include <sstream>
#include <time.h>
#include <exception>
#include <new>

#ifdef _OPENMP
#include <omp.h>
#endif

//#include "debug.h"
//#ifdef HEAP_CHECK
// The heap-checking overridess to new and delete, to find bugs!!!
//#include "debug_mem.h"
//#endif

//#undef new
//#undef delete

