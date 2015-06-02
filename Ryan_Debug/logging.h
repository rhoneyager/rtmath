#pragma once
#pragma warning( disable : 4251 ) // DLL interface - warns on private objects...

#include "defs.h"
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <ostream>
//#include <boost/log/trivial.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <boost/log/sources/channel_feature.hpp>
#include <boost/log/sources/channel_logger.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>

#include "logging_base.h"

namespace Ryan_Debug
{
	namespace log {


		typedef ::boost::log::sources::severity_channel_logger_mt<
			::Ryan_Debug::log::severity_level,     // the type of the severity level
			std::string         // the type of the channel name
		> my_logger_mt;

		BOOST_LOG_ATTRIBUTE_KEYWORD(line_id, "LineID", unsigned int);
		BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", ::Ryan_Debug::log::severity_level);
		BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string);
		BOOST_LOG_ATTRIBUTE_KEYWORD(scope, "Scope", ::boost::log::attributes::named_scope::value_type);
		BOOST_LOG_ATTRIBUTE_KEYWORD(timeline, "Timeline", ::boost::log::attributes::timer::value_type);
		BOOST_LOG_ATTRIBUTE_KEYWORD(channel, "Channel", std::string);
		BOOST_LOG_ATTRIBUTE_KEYWORD(process_id, "ProcessID", ::boost::log::attributes::current_process_id::value_type);
		BOOST_LOG_ATTRIBUTE_KEYWORD(thread_id, "ThreadID", ::boost::log::attributes::current_thread_id::value_type);

		RYAN_DEBUG_DLEXPORT const char*  stringify(severity_level);
		/*
		class logObj;
		typedef boost::shared_ptr<logObj> pLogObj;
		struct iLogObj;
		class RYAN_DEBUG_DLEXPORT logObj {
			logObj();
			boost::scoped_ptr<iLogObj> _impl;
		public:
			static pLogObj makeGlobalLogger(const char* channel);
			static pLogObj makeInstancedLogger(const char* channel, void* obj);

			~logObj();
			std::ostream& log(severity_level);
		};
		*/
	}

}

/// The operator is used for regular stream formatting
RYAN_DEBUG_DLEXPORT std::ostream& operator<< (std::ostream& strm, ::Ryan_Debug::log::severity_level level);
/// The operator is used when putting the severity level to log
RYAN_DEBUG_DLEXPORT ::boost::log::formatting_ostream& operator<< (
		::boost::log::formatting_ostream& strm,
		::boost::log::to_log_manip< ::Ryan_Debug::log::severity_level, ::Ryan_Debug::log::tag::severity > const& manip);


