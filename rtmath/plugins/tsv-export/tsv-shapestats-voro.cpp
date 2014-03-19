/// \brief Provides tsv file IO
#define _SCL_SECURE_NO_WARNINGS

#pragma warning( disable : 4503 ) // decorated length exceeded

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>


#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"

using std::shared_ptr;
using rtmath::ddscat::ddOutput;
using namespace rtmath::registry;

namespace rtmath {
	namespace plugins {
		namespace tsv {

			struct tsv_summary_handle : public rtmath::registry::IOhandler
			{
				tsv_summary_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_VORO) { open(filename, t); }
				virtual ~tsv_summary_handle() {}
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
						"Circum_Sphere_V\tCircum_Sphere_SA\t"
						"Convex_V\tConvex_SA\t"
						"Voronoi_V\tVoronoi_SA\t"
						"as_abs_xy\tas_abs_xz\tas_abs_yz\t"
						"as_rms_xy\tas_rms_xz\tas_rms_yz\t"
						"as_abm_xy\tas_abm_xz\tas_abm_yz\n"
						;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<IOhandler>
				export_tsv_summary_data
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const rtmath::ddscat::stats::shapeFileStats *s)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "summary_data") RTthrow debug::xUnimplementedFunction();
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_summary_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_summary_handle>(new tsv_summary_handle(filename.c_str(), iotype));
				else {
					if (sh->getId() != PLUGINID_VORO) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<tsv_summary_handle>(sh);
				}

				// Initial file creation handles writing the initial header.
				// So, just write the data.
				auto r = s->calcStatsRot(0, 0, 0);
				(*(h->file.get())) << s->_shp->hash().lower << "\t" << s->V_dipoles_const << "\t"
					<< s->Scircum_sphere.V << "\t" << s->Scircum_sphere.SA << "\t"
					<< s->Sconvex_hull.V << "\t" << s->Sconvex_hull.SA << "\t"
					<< s->SVoronoi_hull.V << "\t" << s->SVoronoi_hull.SA << "\t"
					<< r->as_abs(0, 1) << "\t" << r->as_abs(0, 2) << "\t" << r->as_abs(1, 2) << "\t"
					<< r->as_rms(0, 1) << "\t" << r->as_rms(0, 2) << "\t" << r->as_rms(1, 2) << "\t"
					<< r->as_abs_mean(0, 1) << "\t" << r->as_abs_mean(0, 2) << "\t" << r->as_abs_mean(1, 2)
					<< std::endl;
				;

				return h; // Pass back the handle
			}


		}
	}
}
