#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/version.hpp>
#include <boost/date_time.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "../defs.h"
#include "../hash.h"
#include "../registry.h"
#include "../io.h"

#include "arm_info.h"
namespace rtmath
{
	namespace data
	{
		namespace arm
		{
			class arm_scanning_radar_sacr;
			// Reader bindings for netcdf files and database entries.
			class arm_IO_sacr_input_registry {};
			// Used when convertiong file formats and writing database entries.
			class arm_IO_sacr_output_registry {};
			//class arm_info_serialization {};
		}
	}
	namespace registry {
		
		extern template struct IO_class_registry_writer<
			::rtmath::data::arm::arm_scanning_radar_sacr>;

		extern template struct IO_class_registry_reader<
			::rtmath::data::arm::arm_scanning_radar_sacr>;

		extern template class usesDLLregistry<
			::rtmath::data::arm::arm_IO_sacr_input_registry,
			IO_class_registry_reader<::rtmath::data::arm::arm_scanning_radar_sacr> >;

		extern template class usesDLLregistry<
			::rtmath::data::arm::arm_IO_sacr_output_registry,
			IO_class_registry_writer<::rtmath::data::arm::arm_scanning_radar_sacr> >;
		
	}
	namespace data
	{
		namespace arm
		{
			class dataStreamHandler;

			class DLEXPORT_rtmath_data arm_scanning_radar_sacr :
				virtual public boost::enable_shared_from_this<arm_scanning_radar_sacr>,
				virtual public ::rtmath::registry::usesDLLregistry<
					::rtmath::data::arm::arm_IO_sacr_input_registry, 
					::rtmath::registry::IO_class_registry_reader<arm_scanning_radar_sacr> >,
				virtual public ::rtmath::registry::usesDLLregistry<
					::rtmath::data::arm::arm_IO_sacr_output_registry, 
					::rtmath::registry::IO_class_registry_writer<arm_scanning_radar_sacr> >,
				virtual public ::rtmath::io::implementsStandardWriter<arm_scanning_radar_sacr, arm_IO_sacr_output_registry>,
				virtual public ::rtmath::io::implementsStandardReader<arm_scanning_radar_sacr, arm_IO_sacr_input_registry>//,
				//public dataStreamHandler
			{
			public:
				arm_scanning_radar_sacr();
				arm_scanning_radar_sacr(const std::string &filename);
				virtual ~arm_scanning_radar_sacr();

				boost::shared_ptr<const arm_info> info;
				
				/// Distance in m to the center of the range bin (~949 elements)
				Eigen::ArrayXf ranges;
				/// Azimuthal angle (degrees) (~30749 elements)
				Eigen::ArrayXf azimuths;
				/// Elevation angle (degrees) (~30749 elements)
				Eigen::ArrayXf elevations;
				/// Time offsets (in seconds) from start of run (~30749 elements)
				Eigen::ArrayXd time_offsets;
				/// Indicees for start and end of each sweep
				Eigen::ArrayXi sweep_start_ray_index, sweep_end_ray_index;
				/// Stores reflectivity as a function of elevation, azimuth, then range (30749 x 949 elements)
				Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> reflectivity;

				// Skipping read of mean_dipole_velocity, spectral_width, and linear depolarization ratio

			private:
				void _init();
			};
		}
	}
}
