/* error.h - handling routines for all errors that can be thrown */
#pragma once
#include "../defs.h"

#pragma warning( disable : 4996 ) // Deprecated function warning

#include "../Stdafx.h"
#include <iostream>
//#include <boost/exception/all.hpp>
#include "debug_mem.h"

/// \def ERRSTD(x) Defines an exception class that takes no arguments.
#define ERRSTD(x) class DLEXPORT_rtmath_core x : public xError { public: x() : xError() { _setmessage(); } protected: void _setmessage(); }
/// \def ERRSTR(x) Defines an exception class that takes a string as an argument
#define ERRSTR(x) class DLEXPORT_rtmath_core x : public xError { public: x(const char* m) : xError() {_m=m; _setmessage(); } protected: const char *_m; void _setmessage(); }
/// \def ERRSTR(x) Defines an exception class that takes two strings as arguments
#define ERRSTR2(x) class DLEXPORT_rtmath_core x : public xError { public: x(const char* m, const char* n) : xError() {_m=m; _n=n; _setmessage(); } protected: const char *_m, *_n; void _setmessage(); }
/// \def ERRDOU(x) Defines an exception class that takes a double as an argument
#define ERRDOU(x) class DLEXPORT_rtmath_core x : public xError { public: x(double m) : xError() {_m=m; _setmessage(); } protected: double _m; void _setmessage(); }

namespace rtmath
{
	namespace debug
	{
		/// \brief This is the parent error class. Everything inherits from this.
		/// \note Using throw() because MSVC2012 does not have noexcept
		class DLEXPORT_rtmath_core xError : public virtual std::exception //, public virtual boost::exception
		{
		public:
			xError() throw(); 
			virtual ~xError() throw();
			virtual void message(std::string &message) const throw();
			virtual void Display(std::ostream &out = std::cerr) const throw();
			virtual const char* what() const throw();
			bool hasLoc() const;
		public: 
			/**
			 * \brief Handler function that passes error messages
			 *
			 * Allows code to define static functions for
			 * providing error handling to the UI (message boxes and the like).
			 * If not set, sent to stderr.
			 **/
			static void setHandler(void (*func)(const char*));
		protected:
			std::string _message;
			static SHARED_PRIVATE void (*_errHandlerFunc)(const char*);
			virtual void _setmessage() = 0;
			const char* file;
			const char* caller;
			int line;
		};

		/// The throwable assert call failed! It's like assert, but will throw
		ERRSTR(xAssert);

		/// The values passed to the function are nonsensical
		ERRSTR(xBadInput);

		/// \brief The model (typically for optical depth) was called for a value outside
		/// of the expected domain of the function. Results will be nonsensical.
		ERRDOU(xModelOutOfRange);

		/// ddscat data missing for this frequency.
		ERRDOU(xMissingFrequency);

		/// The file that is opened for reading is empty
		ERRSTR(xEmptyInputFile);

		/// File to be opened for reading does not exist
		ERRSTR(xMissingFile);

		/// File / directory to be created already exists and is not the desired type
		ERRSTR(xPathExistsWrongType);

		/// File format is unknown
		ERRSTR(xUnknownFileFormat);

		/// A function has not been defined. Always stops execution.
		ERRSTD(xUnimplementedFunction);

		/// An array went out of bounds.
		ERRSTD(xArrayOutOfBounds);

		/// Attempting to eval a damatrix that is locked and a cached source cannot be found
		ERRSTD(xLockedNotInCache);

		/// Function has been obsoleted.
		ERRSTD(xObsolete);

		/// Function is known to be defective.
		ERRSTD(xDefective);

		/// Singular matrix detected.
		ERRSTD(xSingular);

		/// Duplicate DLL hook
		ERRSTR(xDuplicateHook);

		/// Handle in use
		ERRSTR(xHandleInUse);

		/// Handle not open
		ERRSTR(xHandleNotOpen);

		/// Symbol not found
		ERRSTR2(xSymbolNotFound);

		/// DLL symbol map table invalid
		ERRSTR(xBadFunctionMap);

		/// DLL symbol map table invalid
		ERRSTR(xBadFunctionReturn);

		/// Another hook blocked the load
		ERRSTR2(xBlockedHookLoad);

		/// Another hook blocked unload
		ERRSTR2(xBlockedHookUnload);

		/// Unknown error
		ERRSTD(xOtherError);

	}
}

// Redefine throws so that the location of the code in error is recorded
#ifdef _DEBUG
#ifdef _MSC_FULL_VER
// I need to be careful with MSVC, since this line disagrees with some of the boost headers. So,
// error.h should be loaded after the default system headers
//#define throw (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
#define RTthrow (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
#endif
#ifdef __GNUC__
//#define throw (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#define RTthrow (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#endif
#define TASSERT(x) if(x) ; else RTthrow ::rtmath::debug::xAssert(#x)
#else
#define TASSERT(x) NULL;
#define RTthrow throw
#endif
