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
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-tsv.h"

using std::shared_ptr;
using rtmath::ddscat::ddOutput;
using namespace Ryan_Debug::registry;

namespace rtmath {
	namespace plugins {
		namespace tsv {

			struct tsv_summary_handle : public Ryan_Debug::registry::IOhandler
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
					(*(file.get())) << "Hash\tV_dipoles_const\t"
						"Circum_Sphere_V\tCircum_Sphere_SA\t"
						"Convex_V\tConvex_SA\t"
						"Voronoi_V\tVoronoi_SA\t"
						"Ell_Max_V\tEll_Max_SA\t"
						"Ell_Max_H_V\tEll_Max_H_SA\t"
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
				const boost::shared_ptr<const rtmath::ddscat::stats::shapeFileStats> s)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "summary_data") RDthrow(Ryan_Debug::error::xUnimplementedFunction());
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_summary_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_summary_handle>(new tsv_summary_handle(filename.c_str(), iotype));
				else {
					if (std::string(sh->getId()) != std::string(PLUGINID_VORO)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_summary_handle>(sh);
				}

				// Initial file creation handles writing the initial header.
				// So, just write the data.
				auto r = s->calcStatsRot(0, 0, 0);
				auto &tbl = r->get<0>();
				auto &mat = r->get<1>();
				auto &vec = r->get<2>();
				using namespace rtmath::ddscat::stats;

				(*(h->file.get())) << s->_shp->hash().lower << "\t" << s->V_dipoles_const << "\t"
					<< s->Scircum_sphere.V << "\t" << s->Scircum_sphere.SA << "\t"
					<< s->Sconvex_hull.V << "\t" << s->Sconvex_hull.SA << "\t"
					<< s->SVoronoi_hull.V << "\t" << s->SVoronoi_hull.SA << "\t"
					<< s->Sellipsoid_max.V << "\t" << s->Sellipsoid_max.SA << "\t"
					<< s->Sellipsoid_max_Holly.V << "\t" << s->Sellipsoid_max_Holly.SA << "\t"
					<< mat[rotColDefs::AS_ABS](0, 1) << "\t" << mat[rotColDefs::AS_ABS](0, 2) << "\t" << mat[rotColDefs::AS_ABS](1, 2) << "\t"
					<< mat[rotColDefs::AS_RMS](0, 1) << "\t" << mat[rotColDefs::AS_RMS](0, 2) << "\t" << mat[rotColDefs::AS_RMS](1, 2) << "\t"
					<< mat[rotColDefs::AS_ABS_MEAN](0, 1) << "\t" << mat[rotColDefs::AS_ABS_MEAN](0, 2) << "\t" << mat[rotColDefs::AS_ABS_MEAN](1, 2)
					<< std::endl;
				;

				return h; // Pass back the handle
			}


		}
	}
}
