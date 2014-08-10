// Stdafx-ddscat_base.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently
//#pragma once
#ifndef STDAFX_DDSCAT_BASE_H
#define STDAFX_DDSCAT_BASE_H

#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS

// Define filesystem version 3 for compile issues with older boost
#define BOOST_FILESYSTEM_VERSION 3

#include <algorithm>
#include <cmath>
#include <complex>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#if USE_RYAN_SERIALIZATION
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#endif
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>

#if USE_RYAN_SERIALIZATION
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/complex.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
#endif

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#if USE_RYAN_SERIALIZATION
#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"
#endif

/*
#pragma warning( disable : 4068 ) // ignore GCC pragmas
#pragma warning( disable : 4244 ) // even though that part of code is never reached
#pragma warning( disable : 4146 ) // annoying boost garbage
#pragma warning( disable : 4800 )
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4503 ) // decorated name length exceeded. with boost bimap mpl
*/

/*
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
#include <boost/bimap.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/make_shared.hpp>
#include <boost/units/systems/si.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
*/
//#pragma warning( push )
#endif

