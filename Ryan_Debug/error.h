#pragma once
#include "defs.h"
#include <boost/exception/all.hpp>

#pragma warning( disable : 4996 ) // Deprecated function warning
#pragma warning( disable : 4275 ) // DLL boundary warning
#pragma warning( disable : 4251 ) // DLL interface - warns on private objects...

#include <boost/log/trivial.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <boost/log/sources/channel_feature.hpp>
#include <boost/log/sources/channel_logger.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>

namespace Ryan_Debug {
	namespace error {
		enum severity_level
		{
			debug_3,
			debug_2,
			debug_1,
			normal,
			notification,
			warning,
			error,
			critical
		};

		BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", ::Ryan_Debug::debug::severity_level)
		//BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string)
		BOOST_LOG_ATTRIBUTE_KEYWORD(channel, "Channel", std::string)
	
		typedef boost::error_info<struct tag_file_name, std::string> file_name;
		typedef boost::error_info<struct tag_folder_name, std::string> folder_name;
		typedef boost::error_info<struct tag_symbol_name, std::string> symbol_name;
		typedef boost::error_info<struct tag_pathtype_name, std::pair<std::string, std::string> > path_type_expected;
		typedef boost::error_info<struct tag_range_text, std::string> split_range_name;
		typedef boost::error_info<struct tag_specializer_type, std::string> specializer_type;
//		typedef boost::error_info<struct tag_source_file_name, std::string> source_file_name;
//		typedef boost::error_info<struct tag_source_file_line, int> source_file_line;
//		typedef boost::error_info<struct tag_source_func_sig, std::string> source_func_sig;

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

