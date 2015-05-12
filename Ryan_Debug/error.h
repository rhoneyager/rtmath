#pragma once
#include "defs.h"
#include <boost/exception/all.hpp>

#pragma warning( disable : 4996 ) // Deprecated function warning
#pragma warning( disable : 4275 ) // DLL boundary warning
#pragma warning( disable : 4251 ) // DLL interface - warns on private objects...

namespace Ryan_Debug {
	namespace error {
		
		typedef boost::error_info<struct tag_range_text, std::string> split_range_name;
		typedef boost::error_info<struct tag_specializer_type, std::string> specializer_type;

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

		class RYAN_DEBUG_DLEXPORT xError : public virtual std::exception, public virtual boost::exception
		{
		public:
			xError() throw(); 
			virtual ~xError() throw();
			virtual const char* what() const throw();
		protected:
			//void addLineInfo() const throw();
			const char* getErrLbl() const;
			const char* getErrText() const;
			void setErrLbl(const char* lbl);
			void setErrText(const char* lbl);
		private:
			mutable bool inWhat;
			mutable std::string errLbl;
			mutable std::string errText;
		};

#define ERRSTD(x, mess) class RYAN_DEBUG_DLEXPORT x : public virtual xError { public: x() throw() { setErrLbl(mess); } \
	virtual ~x() throw() {} };


		ERRSTD(xDivByZero, "Divide by zero encountered.");
		ERRSTD(xInvalidRange, "Invalid range.");
		ERRSTD(xAssert, "Assertion failed.");
		ERRSTD(xBadInput, "Bad or nonsensical input.");
		ERRSTD(xNullPointer, "Expected non-null pointer is null.");
		/// \brief The model (typically for optical depth) was called for a value outside
		/// of the expected domain of the function. Results will be nonsensical.
		ERRSTD(xModelOutOfRange, "Model out of range.");
		ERRSTD(xMissingFrequency, "Missing data for this frequency.");
		ERRSTD(xEmptyInputFile, "Input file is empty.");
		ERRSTD(xMissingFolder, "Folder does not exist.");
		ERRSTD(xMissingFile, "Missing file.");
		ERRSTD(xMissingVariable, "Missing variable in saved structure.");
		ERRSTD(xMissingRyan_DebugConf, "Cannot find a Ryan_Debug configuration file.");
		ERRSTD(xFileExists, "File to be opened for writing already exists.");
		ERRSTD(xPathExistsWrongType, "Path exists, but wrong type (e.g. file vs. directory).");
		ERRSTD(xTypeMismatch, "Variable does not have the correct type.");
		ERRSTD(xUnknownFileFormat, "Unknown file format.");
		ERRSTD(xMissingKey, "Key not found in map");
		ERRSTD(xMissingHash, "Cannot find hash to load.");
		ERRSTD(xUnimplementedFunction, "Unimplemented function.");
		ERRSTD(xFallbackTemplate, "Reached point in code that is the fallback template. Missing the appropriate template specialization.");
		ERRSTD(xArrayOutOfBounds, "An array went out of bounds.");
		ERRSTD(xDimensionMismatch, "An array went out of bounds.");
		ERRSTD(xLockedNotInCache, "Object is constant but cannot find cached information requested.");
		ERRSTD(xObsolete, "Function is obsolete. Rewrite code.");
		ERRSTD(xDefective, "Function is defective. Fix it.");
		ERRSTD(xSingular, "Singular matrix detected.");
		ERRSTD(xDuplicateHook, "Attempting to load same DLL twice.");
		ERRSTD(xHandleInUse, "DLL handle is currently in use, but the code wants to overwrite it.");
		ERRSTD(xHandleNotOpen, "Attempting to access unopened DLL handle.");
		ERRSTD(xSymbolNotFound, "Cannot find symbol in DLL.");
		ERRSTD(xBadFunctionMap, "DLL symbol function map table is invalid.");
		ERRSTD(xBadFunctionReturn, "DLL symbol map table is invalid.");
		ERRSTD(xBlockedHookLoad, "Another hook blocked the load operation (for DLLs).");
		ERRSTD(xBlockedHookUnload, "Another DLL depends on the one that you are unloading.");
		ERRSTD(xDLLerror, "Unspecified DLL error.");
		ERRSTD(xUpcast, "Plugin failure: cannot cast base upwards to a derived class provided by a plugin. "
			"Usually means that no matching plugin can be found.");
		ERRSTD(xCannotFindReference, "Cannot find reference in file.");
		ERRSTD(xOtherError, "Unspecified error.");
		ERRSTD(xUnsupportedIOaction, "Unsupported IO action.");
	}
}

#ifdef _MSC_FULL_VER
//#define RDthrow (::Ryan_Debug::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
//#define RDthrow(x) throw x << ::Ryan_Debug::error::source_file_name(__FILE__) << ::Ryan_Debug::error::source_file_line(__LINE__) << ::Ryan_Debug::error::source_func_sig(__FUNCSIG__)
#define RDthrow(x) throw x << ::boost::throw_function(__FUNCSIG__) << ::boost::throw_file(__FILE__) << ::boost::throw_line((int)__LINE__)
#endif
#ifdef __GNUC__
//#define RDthrow(x) throw x << ::Ryan_Debug::error::source_file_name(__FILE__) << ::Ryan_Debug::error::source_file_line(__LINE__) << ::Ryan_Debug::error::source_func_sig(__PRETTY_FUNCTION__) 
//#define RDthrow (::Ryan_Debug::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#define RDthrow(x) throw x << ::boost::throw_function(__PRETTY_FUNCTION__) << ::boost::throw_file(__FILE__) << ::boost::throw_line((int)__LINE__)
#endif

