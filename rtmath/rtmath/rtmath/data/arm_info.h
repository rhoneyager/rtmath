#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
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
			/// Reader bindings for netcdf files and database entries.
			class arm_IO_input_registry {};
			/// Used when convertiong file formats and writing database entries.
			class arm_IO_output_registry {};
//			class arm_info_serialization {};
			/// Database querying
			class arm_query_registry {};

			/// Base class for all ARM data products. Used for casting.
			class DLEXPORT_rtmath_data dataStreamHandler { public: virtual ~dataStreamHandler(); };

			/// \brief This class is used for plugins to register themselves to handle arm_info queries.
			struct DLEXPORT_rtmath_data arm_info_registry
			{
				struct DLEXPORT_rtmath_data arm_info_comp {
					bool operator() (const std::shared_ptr<arm_info>& lhs, 
						const std::shared_ptr<arm_info>& rhs) const;
					bool operator() (const boost::shared_ptr<arm_info>& lhs,
						const boost::shared_ptr<arm_info>& rhs) const;
				};

				/// Language-Integrated Query (LINQ) is not a good idea here, since an external database is used
				class DLEXPORT_rtmath_data arm_info_index
				{
					arm_info_index();
				public:
					std::vector<std::string> instruments, sites, subsites, data_levels, 
						filenames, product_names, stream_names;
					std::vector<boost::posix_time::ptime> discrete_times;
					std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > time_ranges;
				public:
					~arm_info_index();
					static std::shared_ptr<arm_info_index> generate();
					arm_info_index& match_site(const std::string&);
					arm_info_index& match_subsite(const std::string&);
					arm_info_index& time_range(const boost::posix_time::ptime&, const boost::posix_time::ptime&);
					arm_info_index& has_time(const boost::posix_time::ptime&);
					arm_info_index& match_instrument(const std::string&);
					arm_info_index& data_level(const std::string&);
					arm_info_index& filename(const std::string&);
					arm_info_index& filename(const std::vector<std::string>&);
					arm_info_index& product_name(const std::string&);
					arm_info_index& stream_name(const std::string&);

					/// \todo Order collection based on filename
					typedef std::shared_ptr<std::set<boost::shared_ptr<arm_info>, arm_info_comp > > collection;
					std::pair<collection, std::shared_ptr<rtmath::registry::DBhandler> >
						doQuery(std::shared_ptr<rtmath::registry::DBhandler> = nullptr, 
						std::shared_ptr<registry::DB_options> = nullptr) const;

				};

				arm_info_registry();
				virtual ~arm_info_registry();
				/// Module name.
				const char* name;

				enum class updateType { INSERT_ONLY, UPDATE_ONLY }; // , INSERT_AND_UPDATE};

				/// \todo As more database types become prevalent, move this over to 
				/// rtmath::registry and standardize.
				typedef std::function<std::shared_ptr<rtmath::registry::DBhandler>
					(const arm_info_index&, arm_info_index::collection,
					std::shared_ptr<registry::DBhandler>, std::shared_ptr<registry::DB_options>)> queryType;
				typedef std::function<std::shared_ptr<rtmath::registry::DBhandler>
					(const arm_info_index::collection, updateType,
					std::shared_ptr<registry::DBhandler>, std::shared_ptr<registry::DB_options>)> writeType;
				typedef std::function<bool(std::shared_ptr<rtmath::registry::DBhandler>, 
					std::shared_ptr<registry::DB_options>)> matchType;

				/// Get cross-sections from small stats
				queryType fQuery;
				/// Get pfs from small stats
				writeType fInsertUpdate;

				matchType fMatches;
			};
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
		
		extern template class usesDLLregistry<
			::rtmath::data::arm::arm_query_registry,
			::rtmath::data::arm::arm_info_registry >;
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
				virtual public ::rtmath::io::implementsStandardReader<arm_info, arm_IO_input_registry>,
//				virtual public ::rtmath::io::Serialization::implementsSerialization<
//					arm_info, arm_IO_output_registry, arm_IO_input_registry, arm_info_serialization>
				virtual public ::rtmath::registry::usesDLLregistry<
					arm_query_registry, arm_info_registry >
			{
				void _init();
			public:
				arm_info();
				arm_info(const std::string &filename);
				virtual ~arm_info();

				bool operator<(const arm_info &) const;
				bool operator==(const arm_info &) const;
				bool operator!=(const arm_info &) const;

				/// Filename
				std::string filename;

				/// Path to access file (needed in database index)
				std::string filepath;

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
				std::shared_ptr<dataStreamHandler> getHandler() const;

				// These don't really fit the standard io plugin spec, as they refer to database entries.
				// Database search and update semantics are different, and it is much better to do bulk searches 
				// and updates than one-by-one.
				// Of course, the io plugins could be usable, but selecting ranges for read with IO_options is bad, 
				// and handling many writes would either involve code complexity or poor code performance.

				static std::shared_ptr<arm_info_registry::arm_info_index> makeQuery() { return arm_info_registry::arm_info_index::generate(); }
				std::shared_ptr<rtmath::registry::DBhandler> updateEntry(arm_info_registry::updateType,
					std::shared_ptr<rtmath::registry::DBhandler> = nullptr, 
					std::shared_ptr<registry::DB_options> = nullptr) const;
				static arm_info_registry::arm_info_index::collection makeCollection();
				static std::shared_ptr<rtmath::registry::DBhandler> 
					updateCollection(arm_info_registry::arm_info_index::collection, 
					arm_info_registry::updateType, 
					std::shared_ptr<rtmath::registry::DBhandler> = nullptr, 
					std::shared_ptr<registry::DB_options> = nullptr);
			};
		}
	}
}
