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
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-tsv.h"


using std::shared_ptr;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			using namespace Ryan_Debug::registry;
			struct tsv_voro_pts_handle : public Ryan_Debug::registry::IOhandler
			{
				tsv_voro_pts_handle(const char* filename, IOtype t) : IOhandler(PLUGINID) { open(filename, t); }
				virtual ~tsv_voro_pts_handle() {}
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
					//(*(file.get())) << "Hash\tParent Hash\t"
					//	"Flake Type\tPerturbation\tDecimation\t"
					//	"Dipole Spacing (um)\tNumber of Dipoles"
					//	<< std::endl;
					//;
				}
				std::shared_ptr<std::ofstream> file;
			};


			shared_ptr<IOhandler>
				export_tsv_voronoi_depth
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const rtmath::Voronoi::VoronoiDiagram > s)
			{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				std::shared_ptr<tsv_voro_pts_handle> h = Ryan_Debug::registry::construct_handle
					<Ryan_Debug::registry::IOhandler, tsv_voro_pts_handle>(
					sh, PLUGINID, [&](){return std::shared_ptr<tsv_voro_pts_handle>(
					new tsv_voro_pts_handle(filename.c_str(), iotype)); });


				(*(h->file.get())) << s->numPoints() << std::endl;

				auto resDepth = s->calcSurfaceDepth();
				// First three columns are the point, last is the surface depth.

				for (size_t j = 0; j < s->numPoints(); j++)
				{
					auto it = resDepth->block<1, 4>(j, 0);
					(*(h->file.get())) <<  (it)(0) << "\t" << (it)(1) <<
						"\t" << (it)(2) << "\t" << (it)(3) << std::endl;
				}

				return h; // Pass back the handle
			}

		}
	}
}
namespace Ryan_Debug {
	namespace registry {

		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<::rtmath::Voronoi::VoronoiDiagram>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const boost::shared_ptr<const ::rtmath::Voronoi::VoronoiDiagram > s)
		{
			std::string exporttype = opts->exportType();
			if (exporttype == "point_depth_data") return ::rtmath::plugins::tsv::export_tsv_voronoi_depth(sh, opts, s);
			else { RDthrow(Ryan_Debug::error::xUnimplementedFunction()); }
			return nullptr;
		}


	}
}
