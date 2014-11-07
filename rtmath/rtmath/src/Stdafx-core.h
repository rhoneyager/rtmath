// Stdafx-core.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently
//#pragma once

#ifndef STDAFX_CORE_H
#define STDAFX_CORE_H

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

//#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
//#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
//#include <boost/tokenizer.hpp>

#include "../rtmath/defs.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"
//#pragma warning( push )

#endif

