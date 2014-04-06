#include "Stdafx-data.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>
#include <boost/date_time/gregorian/greg_serialize.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

#include "../rtmath/Serialization/serialization_macros.h"

#include "../rtmath/data/arm_info.h"
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
		
	}
	namespace data
	{
		namespace arm
		{
			arm_info::arm_info() { _init(); }
			arm_info::~arm_info() { }

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
				::rtmath::io::Serialization::implementsSerialization<
					arm_info, arm_IO_output_registry, 
					arm_IO_input_registry, arm_info_serialization>::set_sname("rtmath::data::arm::arm_info");
			}

			template<class Archive>
			void arm_info::serialize(Archive &ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("filename", filename);
				ar & boost::serialization::make_nvp("Hash", hash);
				ar & boost::serialization::make_nvp("site", site);
				ar & boost::serialization::make_nvp("subsite", subsite);
				ar & boost::serialization::make_nvp("subsiteFull", subsiteFull);
				ar & boost::serialization::make_nvp("product", product);
				ar & boost::serialization::make_nvp("datalevel", alt);
				ar & boost::serialization::make_nvp("lat", lat);
				ar & boost::serialization::make_nvp("lon", lon);
				ar & boost::serialization::make_nvp("alt", alt);
				ar & boost::serialization::make_nvp("filesize", filesize);
				ar & boost::serialization::make_nvp("startTime", startTime);
				ar & boost::serialization::make_nvp("endTime", endTime);
			}

			EXPORTINTERNAL(::rtmath::data::arm::arm_info::serialize);
		}
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(::rtmath::data::arm::arm_info);

