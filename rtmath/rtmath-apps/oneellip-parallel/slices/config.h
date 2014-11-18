#pragma once

/* This file is designed to allow the code to accomodate some older compilers */

#if __cplusplus >= 201103L

#elif (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)

#elif (_MSC_VER >= 1600) // ver 1600 is vs2010, 1700 is vs2012

#else

#	define override

#endif
