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
#include <Ryan_Debug/error.h>

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
		typedef boost::error_info<struct tag_file_name_b, std::string> file_name_b;
		typedef boost::error_info<struct tag_plugins, 
			std::pair<std::string, std::string> > plugin_types;
		typedef boost::error_info<struct tag_default_file_name, 
			std::string> default_file_name;
		typedef boost::error_info<struct tag_line_number, unsigned long long> line_number;
		typedef boost::error_info<struct tag_line_text, std::string> line_text;
		typedef boost::error_info<struct tag_ref_id, size_t> ref_number;
		typedef boost::error_info<struct tag_folder_name, std::string> folder_name;
		typedef boost::error_info<struct tag_symbol_name, std::string> symbol_name;
		typedef boost::error_info<struct tag_path_type_expected, 
			std::pair<std::string, std::string> > path_type_expected;
		typedef boost::error_info<struct tag_frequency, double> freq;
		typedef boost::error_info<struct tag_temp, double> temp;
		typedef boost::error_info<struct tag_freq_ref_range, 
			std::pair<double, double> > freq_ref_range;
		typedef boost::error_info<struct tag_temp_ref_range, 
			std::pair<double, double> > temp_ref_range;
		typedef boost::error_info<struct tag_range_min, double> rangeMin;
		typedef boost::error_info<struct tag_range_max, double> rangeMax;
		typedef boost::error_info<struct tag_range_span, double> rangeSpan;

		typedef boost::error_info<struct tag_key, std::string> key;
		typedef boost::error_info<struct tag_hash, std::string> hash;
		typedef boost::error_info<struct tag_hash_type, std::string> hashType;
		typedef boost::error_info<struct tag_dll_is_critical, bool> is_Critical; // dll loads
		typedef boost::error_info<struct tag_otherErrorText, std::string> otherErrorText;
		typedef boost::error_info<struct tag_otherErrorCode, long long> otherErrorCode;

#define ERRSTDR(x, mess) class DLEXPORT_rtmath_core x : \
		public virtual ::Ryan_Debug::error::xError \
		{ public: x() throw() { errLbl = mess; } \
		virtual ~x() throw() {} };


		// These are separate classes for error catches.

		ERRSTDR(xAssert, "Assertion failed.");
		ERRSTDR(xBadInput, "Bad or nonsensical input.");
		ERRSTDR(xDivByZero, "Divide by zero encountered.");
		ERRSTDR(xInvalidRange, "Invalid range.");
		ERRSTDR(xNullPointer, "Expected non-null pointer is null.")
		/// \brief The model (typically for optical depth) was called for a value outside
		/// of the expected domain of the function. Results will be nonsensical.
		ERRSTDR(xModelOutOfRange, "Model out of range.");
		ERRSTDR(xMissingFrequency, "Missing data for this frequency.");
		ERRSTDR(xEmptyInputFile, "Input file is empty.");
		ERRSTDR(xMissingFolder, "Folder does not exist.");
		ERRSTDR(xMissingFile, "Missing file.");
		ERRSTDR(xMissingRtmathConf, "Cannot find a rtmath configuration file.");
		ERRSTDR(xFileExists, "File to be opened for writing already exists.");
		ERRSTDR(xPathExistsWrongType, "Path exists, but wrong type (e.g. file vs. directory).");
		ERRSTDR(xUnknownFileFormat, "Unknown file format.");
		ERRSTDR(xMissingKey, "Key not found in map");
		ERRSTDR(xMissingHash, "Cannot find hash to load.");
		ERRSTDR(xUnimplementedFunction, "Unimplemented function.");
		ERRSTDR(xArrayOutOfBounds, "An array went out of bounds.");
		ERRSTDR(xLockedNotInCache, "Object is constant but cannot find cached information requested.");
		ERRSTDR(xObsolete, "Function is obsolete. Rewrite code.");
		ERRSTDR(xDefective, "Function is defective. Fix it.");
		ERRSTDR(xSingular, "Singular matrix detected.");
		ERRSTDR(xDuplicateHook, "Attempting to load same DLL twice.");
		ERRSTDR(xHandleInUse, "DLL handle is currently in use, but the code wants to overwrite it.");
		ERRSTDR(xHandleNotOpen, "Attempting to access unopened DLL handle.");
		ERRSTDR(xSymbolNotFound, "Cannot find symbol in DLL.");
		ERRSTDR(xBadFunctionMap, "DLL symbol function map table is invalid.");
		ERRSTDR(xBadFunctionReturn, "DLL symbol map table is invalid.");
		ERRSTDR(xBlockedHookLoad, "Another hook blocked the load operation (for DLLs).");
		ERRSTDR(xBlockedHookUnload, "Another DLL depends on the one that you are unloading.");
		ERRSTDR(xDLLerror, "Unspecified DLL error.");
		ERRSTDR(xUpcast, "Plugin failure: cannot cast base upwards to a derived class provided by a plugin. "
			"Usually means that no matching plugin can be found.");
		ERRSTDR(xCannotFindReference, "Cannot find reference in file.")
		ERRSTDR(xOtherError, "Unspecified error.");
		ERRSTDR(xUnsupportedIOaction, "Unsupported IO action.");

	}
}

// Redefine throws so that the location of the code in error is recorded

//#ifdef _DEBUG
#ifdef _MSC_FULL_VER
#define RTthrow RDthrow
#endif
#ifdef __GNUC__
#define RTthrow RDthrow 
#endif
#define TASSERT(x) if(x) ; else RTthrow ::rtmath::debug::xAssert(#x)
//#else
//#define TASSERT(x) NULL;
//#define RTthrow throw
//#endif
