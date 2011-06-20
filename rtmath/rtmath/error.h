/* error.h - handling routines for all errors that can be thrown */
#pragma once

#include "Stdafx.h"
#include <iostream>

// Redefine throws so that the location of the code in error is recorded
// TODO: rewrite so that it works with non-MSVC compilers
#ifdef _DEBUG
#ifdef _MSC_FULL_VER
#define throw (rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
#endif
#ifdef __GNUC__
#define throw (rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#endif
#define TASSERT(x) if(x) ; else throw rtmath::debug::xAssert(#x)
#else
#define TASSERT(x) NULL;
#endif

#define ERRSTD : public xError { protected: void _setmessage(); };
#define ERRSTR(x) class x : public xError { public: x (const char* m) : xError() {_m=m;} protected: const char *_m; void _setmessage(); }
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
		class xBadInput ERRSTD;

		// The file that is opened for reading is empty
		ERRSTR(xEmptyInputFile);

		// A function has not been defined. Always stops execution.
		class xUnimplementedFunction ERRSTD;

	}; // end debug
}; // end rtmath
