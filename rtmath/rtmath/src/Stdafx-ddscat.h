// Stdafx-ddscat.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently
//#pragma once

#ifndef STDAFX_DDSCAT_H
#define STDAFX_DDSCAT_H

#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS

// Define filesystem version 3 for compile issues with older boost
#define BOOST_FILESYSTEM_VERSION 3

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "../rtmath/registry.h"
#include "../rtmath/error/error.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/defs.h"

#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#if USE_RYAN_SERIALIZATION
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
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

#include <Eigen/Core>
#include <Eigen/Dense>


/*
#pragma warning( disable : 4068 ) // ignore GCC pragmas
#pragma warning( disable : 4244 ) // even though that part of code is never reached
#pragma warning( disable : 4146 ) // annoying boost garbage
#pragma warning( disable : 4800 )
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4503 ) // decorated name length exceeded. with boost bimap mpl
*/

//#pragma warning( push )
#endif

