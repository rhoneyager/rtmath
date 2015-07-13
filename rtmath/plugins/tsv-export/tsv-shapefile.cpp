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
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-tsv.h"


using std::shared_ptr;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			using namespace Ryan_Debug::registry;
			struct tsv_shp_handle : public Ryan_Debug::registry::IOhandler
			{
				tsv_shp_handle(const char* filename, IOtype t) : IOhandler(PLUGINID) { open(filename, t); }
				virtual ~tsv_shp_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RDthrow(Ryan_Debug::error::xOtherError());
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RDthrow(Ryan_Debug::error::xFileExists());
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
					(*(file.get())) << "Hash\tParent Hash\t"
						"Flake Type\tPerturbation\tDecimation\t"
						"Dipole Spacing (um)\tNumber of Dipoles"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};


			// in separate file
			shared_ptr<IOhandler>
				export_tsv_shape_points
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile > s);

			shared_ptr<IOhandler>
				export_tsv_shape_data
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile > s)
			{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				using std::shared_ptr;
				std::shared_ptr<tsv_shp_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<tsv_shp_handle>(new tsv_shp_handle(filename.c_str(), iotype));
				}
				else {
					if (std::string(sh->getId()) != std::string(PLUGINID)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_shp_handle>(sh);
				}

				std::string sParent, sPert, sDec, sType;
				if (s->tags.count("flake_reference")) sParent = s->tags.at("flake_reference");
				if (s->tags.count("decimation")) sDec = s->tags.at("decimation");
				if (s->tags.count("flake_classification")) sType = s->tags.at("flake_classification");
				if (s->tags.count("perturbation")) sPert = s->tags.at("perturbation");


				(*(h->file.get())) << s->hash().lower << "\t" << sParent << "\t"
					<< sType << "\t" << sPert << "\t" << sDec << "\t"
					<< s->standardD << "\t" << s->numPoints
					<< std::endl;

				return h; // Pass back the handle
			}

		}
	}
}
namespace Ryan_Debug {
	namespace registry {
		using rtmath::ddscat::ddOutput;

		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<rtmath::ddscat::shapefile::shapefile>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile > s)
		{
			std::string exporttype = opts->exportType();
			if (exporttype == "shape_data") return ::rtmath::plugins::tsv::export_tsv_shape_data(sh, opts, s);
			else if (exporttype == "shape_points") return ::rtmath::plugins::tsv::export_tsv_shape_points(sh, opts, s);
			else { RDthrow(Ryan_Debug::error::xUnimplementedFunction()); }
			return nullptr;
		}


	}
}
