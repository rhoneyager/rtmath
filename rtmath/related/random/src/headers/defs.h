#pragma once

/*!
	\todo Need to add shared / static flag in CMakeLists.txt.

	\todo Switch to internal serialization

	\def EXPORTING_TMATRIX_RANDOM
	Flag for header reuse to toggle export / import.
	Only set in this packages cpp files (internal use only).

	\def SHARED_TMATRIX_RANDOM is set when building using shared libraries. 

	\def DLEXPORT_TMATRIX_RANDOM
	Indicates that the given symbol is exported from the 
	C++ library for use by other software. To clients, it 
	looks like dllimport.
	*/

#ifdef EXPORTING_TMATRIX_RANDOM
	#ifdef _MSC_FULL_VER
		#ifdef SHARED_tmatrix_cpp
			#define DLEXPORT_TMATRIX_RANDOM __declspec(dllexport)
		#else
			#define DLEXPORT_TMATRIX_RANDOM
		#endif
	#else
		#define DLEXPORT_TMATRIX_RANDOM
	#endif
#else
	#ifdef _MSC_FULL_VER
		#ifdef SHARED_tmatrix_cpp
			#define DLEXPORT_TMATRIX_RANDOM __declspec(dllimport)
		#else
			#define DLEXPORT_TMATRIX_RANDOM
		#endif
	#else
		#define DLEXPORT_TMATRIX_RANDOM
	#endif
#endif

//#ifdef _MSC_FULL_VER
//#include "Ryan.Tmatrix.manifest.h"
//#endif

