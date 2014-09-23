/// \brief Provides tsv file IO
#define _SCL_SECURE_NO_WARNINGS


#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>


#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"


using std::shared_ptr;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			using namespace rtmath::registry;
			struct tsv_shp_handle : public rtmath::registry::IOhandler
			{
				tsv_shp_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_ARS) { open(filename, t); }
				virtual ~tsv_shp_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RTthrow debug::xOtherError();
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RTthrow("File already exists");
					case IOtype::TRUNCATE:
						file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::trunc));
						writeHeader();
						break;
					case IOtype::READWRITE:
						{
							bool e = false;
							if (exists(path(filename))) e = true;
							file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::app));
							if (!e) writeHeader(); // If the file had to be created, give it a header
						}
						break;
					}
				}
				void writeHeader()
				{
					(*(file.get())) << "Hash\tV_dipoles_const\t"
						"aeff_um\tNumber of Rotations\t"
						"min_as_abs_xy\tmin_as_abs_xz\tmin_as_abs_yz\t"
						"max_as_abs_xy\tmax_as_abs_xz\tmax_as_abs_yz\t"
						"mean_as_abs_xy\tmean_as_abs_xz\tmean_as_abs_yz\t"
						"skewness_as_abs_xy\tskewness_as_abs_xz\tskewness_as_abs_yz\t"
						"kurtosis_as_abs_xy\tkurtosis_as_abs_xz\tkurtosis_as_abs_yz\t"
						"variance_as_abs_xy\tvariance_as_abs_xz\tvariance_as_abs_yz"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

	namespace registry {
		using rtmath::ddscat::ddOutput;

		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<rtmath::ddscat::shapefile::shapefile>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const rtmath::ddscat::shapefile::shapefile *s)
		{
			std::string exporttype = opts->exportType();
			if (exporttype == "shape_data") return ::rtmath::plugins::tsv::export_tsv_shapefile_data(sh, opts, s);
			else { RTthrow debug::xUnimplementedFunction(); }
			return nullptr;
		}


	}
}
