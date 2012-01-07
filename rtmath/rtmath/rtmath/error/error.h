/* error.h - handling routines for all errors that can be thrown */
#pragma once

#include "../Stdafx.h"
#include <iostream>
#include "debug_mem.h"

// Redefine throws so that the location of the code in error is recorded
// TODO: rewrite so that it works with non-MSVC compilers
#ifdef _DEBUG
#ifdef _MSC_FULL_VER
//#define throw (rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
#endif
#ifdef __GNUC__
#define throw (rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#endif
#define TASSERT(x) if(x) ; else throw rtmath::debug::xAssert(#x)
#else
#define TASSERT(x) NULL;
#endif

//#define ERRSTD : public xError { protected: void _setmessage(); };
#define ERRSTD(x) class x : public xError { public: x() : xError() { _setmessage(); } protected: void _setmessage(); }
#define ERRSTR(x) class x : public xError { public: x(const char* m) : xError() {_m=m; _setmessage(); } protected: const char *_m; void _setmessage(); }
namespace rtmath
{
	namespace debug
	{
		class xError
		{
			// This is the parent error class. Everything inherits from this.
		public:
			xError();
			virtual ~xError();
			virtual void message(std::string &message) const;
			virtual void Display() const;
			bool hasLoc() const;
		public: // The static functions for:
			// providing error handling to the UI (message boxes and the like)
			// if not set, sent to stderr.
			static void setHandler(void (*func)(const char*));
		protected:
			std::string _message;
			static void (*_errHandlerFunc)(const char*);
			virtual void _setmessage() = 0;
			const char* file;
			const char* caller;
			int line;
		};

		// The throwable assert call failed! It's like assert, but will throw
		ERRSTR(xAssert);

		// The values passed to the function are nonsensical
		// TODO: extend to also take a string for an error message
		ERRSTD(xBadInput);

		// The file that is opened for reading is empty
		ERRSTR(xEmptyInputFile);

		// A function has not been defined. Always stops execution.
		ERRSTD(xUnimplementedFunction);

		// An array went out of bounds.
		ERRSTD(xArrayOutOfBounds);

		// Attempting to eval a damatrix that is locked and a cached source cannot be found
		ERRSTD(xLockedNotInCache);

		// Function has been obsoleted.
		ERRSTD(xObsolete);

		// Function is known to be defective.
		ERRSTD(xDefective);

		// Singular matrix detected.
		ERRSTD(xSingular);

		// A convenient way to mark a class as obsolete and prevent execution 
		// of functions that use the obsolete class is to make the class derive from obsoleted.
		class obsoleted
		{
		public:
			obsoleted();
		};

		class defective
		{
		public:
			defective();
		};

	}; // end debug
}; // end rtmath
