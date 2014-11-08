/* error.h - handling routines for all errors that can be thrown */
#pragma once
#include "../defs.h"

#pragma warning( disable : 4996 ) // Deprecated function warning
#pragma warning( disable : 4275 ) // DLL boundary warning

#include "../Stdafx.h"
#include <iostream>
#include <boost/exception/all.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <boost/log/sources/channel_feature.hpp>
#include <boost/log/sources/channel_logger.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#ifdef HEAP_CHECK
#include "debug_mem.h"
#endif


/// \def ERRSTD(x) Defines an exception class that takes no arguments.
#define ERRSTD(x) class DLEXPORT_rtmath_core x : public xError { public: x() : xError() { _setmessage(); } protected: void _setmessage(); }
/// \def ERRSTR(x) Defines an exception class that takes a string as an argument
#define ERRSTR(x) class DLEXPORT_rtmath_core x : public xError { public: x(const char* m) : xError() {_m=m; _setmessage(); } protected: const char *_m; void _setmessage(); }
/// \def ERRSTR(x) Defines an exception class that takes two strings as arguments
#define ERRSTR2(x) class DLEXPORT_rtmath_core x : public xError { public: x(const char* m, const char* n) : xError() {_m=m; _n=n; _setmessage(); } protected: const char *_m, *_n; void _setmessage(); }
/// \def ERRDOU(x) Defines an exception class that takes a double as an argument
#define ERRDOU(x) class DLEXPORT_rtmath_core x : public xError { public: x(double m) : xError() {_m=m; _setmessage(); } protected: double _m; void _setmessage(); }

namespace blog = boost::log;

namespace rtmath
{
	namespace debug
	{
		enum severity_level
		{
			normal,
			notification,
			warning,
			error,
			critical
		};

		BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", ::rtmath::debug::severity_level)
		BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string)
		typedef ::boost::log::sources::severity_channel_logger_mt<
			severity_level,     // the type of the severity level
			std::string         // the type of the channel name
		> my_logger_mt;

		// Where the errors occur
		//typedef boost::error_info<struct tag_file_name, std::string> src_file;
		//typedef boost::error_info<struct tag_file_name, int> src_line;
		//typedef boost::error_info<struct tag_file_name, std::string> func_name;

		//typedef boost::tuple<boost::errinfo_api_function, boost::errinfo_errno> clib_failure;

		typedef boost::error_info<struct tag_file_name, std::string> file_name;
		typedef boost::error_info<struct tag_file_name, std::string> default_file_name;
		typedef boost::error_info<struct tag_file_name, std::string> folder_name;
		typedef boost::error_info<struct tag_file_name, std::string> symbol_name;
		typedef boost::error_info<struct tag_file_name, std::pair<std::string, std::string> > path_type_expected;
		typedef boost::error_info<struct tag_file_name, double> freq;
		typedef boost::error_info<struct tag_file_name, double> temp;
		typedef boost::error_info<struct tag_file_name, std::pair<double, double> > freq_ref_range;
		typedef boost::error_info<struct tag_file_name, std::pair<double, double> > temp_ref_range;
		typedef boost::error_info<struct tag_file_name, std::string> hash;
		typedef boost::error_info<struct tag_file_name, bool> is_Critical; // dll loads
		typedef boost::error_info<struct tag_file_name, std::string> otherErrorText;
		typedef boost::error_info<struct tag_file_name, long long> otherErrorCode;

		//typedef boost::error_info<struct tag_file_name, std::string> var_name;

		/// \brief This is the parent error class. Everything inherits from this.
		/// \note Using throw() because MSVC2012 does not have noexcept
		class DLEXPORT_rtmath_core xError : public virtual std::exception, public virtual boost::exception
		{
		public:
			xError() throw(); 
			xError(const std::string&) throw();
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
			virtual void _setmessage();
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

		/// Folder does not exist
		ERRSTR(xMissingFolder);

		/// File to be opened for reading does not exist
		ERRSTR(xMissingFile);

		/// File to be opened for writing already exists
		ERRSTR(xFileExists);

		/// File / directory to be created already exists and is not the desired type
		ERRSTR(xPathExistsWrongType);

		/// File format is unknown
		ERRSTR(xUnknownFileFormat);

		/// Cannot find hash to load
		ERRSTR2(xMissingHash);

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

		/// DLL function error (unspecified)
		ERRSTR(xDLLerror);


		/// Cannot cast upwards to a derived class (usually for plugin handling)
		ERRSTR2(xUpcast);

		/// Unknown error
		ERRSTD(xOtherError);

		/// Unsupported IO action
		ERRSTR(xUnsupportedIOaction);


		namespace memcheck {
			extern DLEXPORT_rtmath_core const char* __file__;
			extern DLEXPORT_rtmath_core size_t __line__;
			extern DLEXPORT_rtmath_core const char* __caller__;
			extern DLEXPORT_rtmath_core bool enabled;
			bool DLEXPORT_rtmath_core __Track(int option, void* p, size_t size, const char* file, int line, const char* caller);
			inline bool setloc(const char* _file, int _line, const char* _caller)
			{
				rtmath::debug::memcheck::__file__ = _file;
				rtmath::debug::memcheck::__line__ = _line;
				rtmath::debug::memcheck::__caller__ = _caller;
				return false;
			}
		}

	}
}

// Redefine throws so that the location of the code in error is recorded

//#ifdef _DEBUG
#ifdef _MSC_FULL_VER
#define RTthrow (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
#endif
#ifdef __GNUC__
#define RTthrow (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#endif
#define TASSERT(x) if(x) ; else RTthrow ::rtmath::debug::xAssert(#x)
//#else
//#define TASSERT(x) NULL;
//#define RTthrow throw
//#endif
