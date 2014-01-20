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
//#include <memory>
//#include <new>
#include <set>
#include <sstream>
#include <string>
#include <vector>

/*
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
*/
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
/*
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
*/
//#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

//#pragma warning( push )

#endif

