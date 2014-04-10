#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
//#include <boost/serialization/version.hpp>
#include <boost/date_time.hpp>
#include "../defs.h"
#include "../hash.h"
#include "../registry.h"
#include "../io.h"

namespace rtmath
{
	namespace data
	{
		namespace arm
		{
			class arm_info;
			// Reader bindings for netcdf files and database entries.
			class arm_IO_input_registry {};
			// Used when convertiong file formats and writing database entries.
			class arm_IO_output_registry {};
//			class arm_info_serialization {};
			class dataStreamHandler {};
		}
	}
	namespace registry {
		
		extern template struct IO_class_registry_writer<
			::rtmath::data::arm::arm_info>;

		extern template struct IO_class_registry_reader<
			::rtmath::data::arm::arm_info>;

		extern template class usesDLLregistry<
			::rtmath::data::arm::arm_IO_input_registry,
			IO_class_registry_reader<::rtmath::data::arm::arm_info> >;
			//::rtmath::ddscat::shapefile::shapefile_IO_class_registry>;

		extern template class usesDLLregistry<
			::rtmath::data::arm::arm_IO_output_registry,
			IO_class_registry_writer<::rtmath::data::arm::arm_info> >;
		
	}
	namespace data
	{
		namespace arm
		{
			class dataStreamHandler;

			/** \brief Ascertains information about a data file from ARM.
			*
			* This class reads a file from ARM, determines the type of instrument used, 
			* the site, the datastream and the time range. It is intended to be very useful 
			* for establishing coverage information.
			*
			* Reads ARM files and database entries.
			* Writes / exports database and tsv information.
			**/
			class DLEXPORT_rtmath_data arm_info :
				virtual public boost::enable_shared_from_this<arm_info>,
				virtual public ::rtmath::registry::usesDLLregistry<
					::rtmath::data::arm::arm_IO_input_registry, 
					::rtmath::registry::IO_class_registry_reader<arm_info> >,
				virtual public ::rtmath::registry::usesDLLregistry<
					::rtmath::data::arm::arm_IO_output_registry, 
					::rtmath::registry::IO_class_registry_writer<arm_info> >,
				virtual public ::rtmath::io::implementsStandardWriter<arm_info, arm_IO_output_registry>,
				virtual public ::rtmath::io::implementsStandardReader<arm_info, arm_IO_input_registry>
//				virtual public ::rtmath::io::Serialization::implementsSerialization<
//					arm_info, arm_IO_output_registry, arm_IO_input_registry, arm_info_serialization>
			{
			public:
				arm_info();
				arm_info(const std::string &filename);
				virtual ~arm_info();

				/// Filename
				std::string filename;

				/// ARM main site (SGP, TWP, NSA, ...)
				std::string site;
				/// ARM subsite (C1, ...)
				std::string subsite;
				std::string subsiteFull;

				/// Data product
				std::string product;
				std::string productFull;
				/// Data stream
				std::string stream;

				/// Data level
				std::string datalevel;

				/// Instrument coordinates (degrees, degrees, meters above sea level)
				float lat, lon, alt;

				/// Start time
				boost::posix_time::ptime startTime;
				/// End time
				boost::posix_time::ptime endTime;

				/// File size (bytes)
				size_t filesize;

				// File hash (disabled since some of these files are large)
				//HASH_t hash;

				/// Returns a unique folder location to hold this file
				std::string indexLocation() const;

				/// \brief Based on the datastream, if there is a handler, then return a shared pointer to the 
				/// appropriate stream analysis type.
				boost::shared_ptr<dataStreamHandler> getHandler() const;

			private:
				void _init();
//				friend class ::boost::serialization::access;
//				template<class Archive>
//				void serialize(Archive & ar, const unsigned int version);
			};
		}
	}
}
//BOOST_CLASS_EXPORT_KEY(::rtmath::data::arm::arm_info);
//BOOST_CLASS_VERSION(::rtmath::data::arm::arm_info, 0);

