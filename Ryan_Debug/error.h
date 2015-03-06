#pragma once
#include "defs.h"
#include <boost/exception/all.hpp>

#pragma warning( disable : 4996 ) // Deprecated function warning
#pragma warning( disable : 4275 ) // DLL boundary warning


namespace Ryan_Debug {
	namespace error {
		typedef boost::error_info<struct tag_file_name, std::string> file_name;
		typedef boost::error_info<struct tag_folder_name, std::string> folder_name;
		typedef boost::error_info<struct tag_symbol_name, std::string> symbol_name;
		typedef boost::error_info<struct tag_pathtype_name, std::pair<std::string, std::string> > path_type_expected;
		typedef boost::error_info<struct tag_range_text, std::string> split_range_name;
		typedef boost::error_info<struct tag_specializer_type, std::string> specializer_type;
		typedef boost::error_info<struct tag_source_file_name, std::string> source_file_name;
		typedef boost::error_info<struct tag_source_file_line, int> source_file_line;
		typedef boost::error_info<struct tag_source_func_sig, std::string> source_func_sig;

		class RYAN_DEBUG_DLEXPORT xError : public virtual std::exception, public virtual boost::exception
		{
		public:
			xError() throw(); 
			virtual ~xError() throw();
			virtual const char* what() const throw();
		protected:
			void addLineInfo() const throw();
			mutable std::string errText;
		};

#define ERRSTD(x) class RYAN_DEBUG_DLEXPORT x : public virtual xError { public: x() throw() {} \
	virtual ~x() throw() {} virtual const char* what() const throw(); };

		ERRSTD(xDivByZero);
		ERRSTD(xInvalidRange);
	}
}

#ifdef _MSC_FULL_VER
//#define RDthrow (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : throw 
#define RDthrow(x) throw x << ::Ryan_Debug::error::source_file_name(__FILE__) << ::Ryan_Debug::error::source_file_line(__LINE__) << ::Ryan_Debug::error::source_func_sig(__FUNCSIG__)
#endif
#ifdef __GNUC__
#define RDthrow(x) throw x << ::Ryan_Debug::error::source_file_name(__FILE__) << ::Ryan_Debug::error::source_file_line(__LINE__) << ::Ryan_Debug::error::source_func_sig(__PRETTY_FUNCTION__) 
//#define RDthrow (::rtmath::debug::memcheck::setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : throw 
#endif

