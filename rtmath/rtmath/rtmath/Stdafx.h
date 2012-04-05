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
#include <unordered_map>
#include <boost/unordered_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/units/systems/si.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/math/constants/constants.hpp>
#include <set>
#include <cstdlib>
#include <cstdio>
#include <cstring>
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

