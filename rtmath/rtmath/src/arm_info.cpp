#include "Stdafx-data.h"
#include <sstream>

#include "../rtmath/data/arm_info.h"

#include "../rtmath/data/arm_scanning_radar_sacr.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace registry {
		
		template struct IO_class_registry_writer<
			::rtmath::data::arm::arm_info>;

		template struct IO_class_registry_reader<
			::rtmath::data::arm::arm_info>;

		template class usesDLLregistry<
			::rtmath::data::arm::arm_IO_input_registry,
			IO_class_registry_reader<::rtmath::data::arm::arm_info> >;

		template class usesDLLregistry<
			::rtmath::data::arm::arm_IO_output_registry,
			IO_class_registry_writer<::rtmath::data::arm::arm_info> >;

		template class usesDLLregistry<
			::rtmath::data::arm::arm_query_registry,
			::rtmath::data::arm::arm_info_registry >;
		
	}
	namespace data
	{
		namespace arm
		{
			dataStreamHandler::~dataStreamHandler() {}

			arm_info::arm_info() { _init(); }
			arm_info::~arm_info() { }
			bool arm_info::operator!=(const arm_info &rhs) const { return !operator==(rhs); }
			bool arm_info::operator==(const arm_info &rhs) const { return filename == rhs.filename; }
			bool arm_info::operator<(const arm_info &rhs) const { return filename < rhs.filename; }

			arm_info::arm_info(const std::string &filename)
			{
				_init();
				read(filename);
				this->filename = filename;
			}

			void arm_info::_init()
			{
				filesize = 0;
				lat = 0;
				lon = 0;
				alt = 0;
				//filename = "";
//				::rtmath::io::Serialization::implementsSerialization<
//					arm_info, arm_IO_output_registry, 
//					arm_IO_input_registry, arm_info_serialization>::set_sname("rtmath::data::arm::arm_info");
			}

			boost::shared_ptr<dataStreamHandler> arm_info::getHandler() const
			{
				boost::shared_ptr<dataStreamHandler> res;
				//if (product == "wsacr") res = boost::shared_ptr<dataStreamHandler>(
				//	new arm_scanning_radar_sacr(filename));
				RTthrow debug::xUnimplementedFunction();
				return res;
			}

			std::string arm_info::indexLocation() const
			{
				/* Follows the pattern:
				 * - product
				 * - stream
				 * - year
				 * - site
				 * - subsite
				 * - datalevel
				 */
				int year = 0;
				// Year gets converted from startTime
				year = startTime.date().year();

				std::ostringstream out;
				out << product << "/";
				if (stream.size()) out << stream << "/";
				out << year << "/" << site << "/"
					<< subsite << "/" << datalevel;
				std::string res = out.str();
				return res;
			}

			arm_info_registry::arm_info_registry() : name(nullptr), fQuery(nullptr) {}

			arm_info_registry::~arm_info_registry() {}

			arm_info_registry::arm_info_index::arm_info_index() {}
			arm_info_registry::arm_info_index::~arm_info_index() {}
			boost::shared_ptr<arm_info_registry::arm_info_index> 
				arm_info_registry::arm_info_index::generate()
			{
				boost::shared_ptr<arm_info_registry::arm_info_index> res
					(new arm_info_index());
				return res;
			}

			arm_info_registry::arm_info_index& arm_info_registry::arm_info_index::match_site(const std::string& s)
			{
				sites.push_back(s);
				return *this;
			}

			arm_info_registry::arm_info_index& arm_info_registry::arm_info_index::match_subsite(const std::string& s)
			{
				subsites.push_back(s);
				return *this;
			}
			arm_info_registry::arm_info_index& arm_info_registry::arm_info_index::time_range(const boost::posix_time::ptime& b, const boost::posix_time::ptime& e)
			{
				time_ranges.push_back(std::pair<boost::posix_time::ptime, boost::posix_time::ptime>(b,e));
				return *this;
			}
			arm_info_registry::arm_info_index& arm_info_registry::arm_info_index::has_time(const boost::posix_time::ptime& t)
			{
				discrete_times.push_back(t);
				return *this;
			}
			arm_info_registry::arm_info_index& arm_info_registry::arm_info_index::match_instrument(const std::string& s)
			{
				instruments.push_back(s);
				return *this;
			}
			arm_info_registry::arm_info_index& arm_info_registry::arm_info_index::data_level(const std::string& s)
			{
				data_levels.push_back(s);
				return *this;
			}

			arm_info_registry::arm_info_index::collection arm_info_registry::arm_info_index::doQuery() const
			{
				collection c(new std::set<boost::shared_ptr<arm_info> >());

				auto hooks = ::rtmath::registry::usesDLLregistry<arm_query_registry, arm_info_registry >::getHooks();
				for (const auto &h : *(hooks.get()))
				{
					if (!h.fQuery) continue;
					h.fQuery(*this, c);
				}

				return c;
			}

			arm_info_registry::arm_info_index::collection arm_info::makeCollection()
			{
				return arm_info_registry::arm_info_index::collection(new std::set<boost::shared_ptr<arm_info> >());
			}
			
			void arm_info::updateEntry(arm_info_registry::updateType t) const
			{
				auto c = makeCollection();
				c->insert(boost::shared_ptr<arm_info>(new arm_info(*this)));
				updateCollection(c, t);
			}

			void arm_info::updateCollection(arm_info_registry::arm_info_index::collection c, 
				arm_info_registry::updateType t)
			{
				auto hooks = ::rtmath::registry::usesDLLregistry<arm_query_registry, arm_info_registry >::getHooks();
				for (const auto &h : *(hooks.get()))
				{
					if (!h.fInsertUpdate) continue;
					h.fInsertUpdate(c, t);
				}
			}

			
		}
	}
}
