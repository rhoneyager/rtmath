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
			struct tsv_shp_pts_handle : public Ryan_Debug::registry::IOhandler
			{
				tsv_shp_pts_handle(const char* filename, IOtype t) : IOhandler(PLUGINID) { open(filename, t); }
				virtual ~tsv_shp_pts_handle() {}
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
				export_tsv_shape_points
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile > s)
			{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				using std::shared_ptr;
				std::shared_ptr<tsv_shp_pts_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<tsv_shp_pts_handle>(new tsv_shp_pts_handle(filename.c_str(), iotype));
				}
				else {
					if (std::string(sh->getId()) != std::string(PLUGINID_SHP2)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_shp_pts_handle>(sh);
				}


				(*(h->file.get())) << s->numPoints << std::endl;

				std::vector<long> oi(s->numPoints * 7);

				for (size_t j = 0; j < s->numPoints; j++)
				{
					long point = s->latticeIndex(j);
					auto it = s->latticePts.block<1, 3>(j, 0);
					auto ot = s->latticePtsRi.block<1, 3>(j, 0);
					(*(h->file.get())) <<  (it)(0) << "\t" << (it)(1) <<
						"\t" << (it)(2) << "\t" << ot(0) << std::endl;
					//oi[j * 7 + 0] = point;
					//oi[j * 7 + 1] = (long)(it)(0);
					//oi[j * 7 + 2] = (long)(it)(1);
					//oi[j * 7 + 3] = (long)(it)(2);
					//oi[j * 7 + 4] = (long)(ot)(0);
					//oi[j * 7 + 5] = (long)(ot)(1);
					//oi[j * 7 + 6] = (long)(ot)(2);

					//out << "\t" << i << "\t";
					//out << (it)(0) << "\t" << (it)(1) << "\t" << (it)(2) << "\t";
					//out << (ot)(0) << "\t" << (ot)(1) << "\t" << (ot)(2);
					//out << endl;
				}

				//(*(h->file.get())) << s->hash().lower << "\t" << sParent << "\t"
				//	<< sType << "\t" << sPert << "\t" << sDec << "\t"
				//	<< s->standardD << "\t" << s->numPoints
				//	<< std::endl;

				return h; // Pass back the handle
			}

		}
	}
}
