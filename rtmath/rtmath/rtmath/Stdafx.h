// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once

#pragma warning( disable : 4068 ) // ignore GCC pragmas
#pragma warning( disable : 4244 ) // even though that part of code is never reached
#pragma warning( disable : 4146 ) // annoying boost garbage
#pragma warning( disable : 4800 )
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4503 ) // decorated name length exceeded. with boost bimap mpl
#include <algorithm>
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <complex>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <new>
#include <set>
#include <sstream>
#include <string>
#include <time.h>
#include <unordered_map>
#include <vector>

// Define filesystem version 3 for compile issues with older boost
#define BOOST_FILESYSTEM_VERSION 3

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/bimap.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <boost/math/constants/constants.hpp>
#include <boost/make_shared.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/units/systems/si.hpp>
#include <boost/unordered_map.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

//#pragma warning( push )

/*
#include "TGraph.h"
#include "TF1.h"
#include "TCanvas.h"
#include "TAxis.h"
#include "TNamed.h"
#include "TGraph2D.h"
#include "TStyle.h"
#include "TH2.h"
*/
//#pragma warning( pop ) 

//#ifdef _OPENMP
//#include <omp.h>
//#endif

/*
#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>
#endif
*/

//#include "debug.h"
//#ifdef HEAP_CHECK
// The heap-checking overridess to new and delete, to find bugs!!!
//#include "debug_mem.h"
//#endif

//#undef new
//#undef delete

