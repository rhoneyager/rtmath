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
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"


namespace rtmath {
	namespace plugins {
		namespace tsv {
			using std::shared_ptr;
			using namespace rtmath::ddscat;
			using rtmath::ddscat::ddOutput;
			using rtmath::ddscat::rotations;
			using namespace rtmath::registry;

			struct tsv_ddoutput_iso_handle : public rtmath::registry::IOhandler
			{
				tsv_ddoutput_iso_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_DDISO) { open(filename, t); }
				virtual ~tsv_ddoutput_iso_handle() {}
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
					(*(file.get())) << "Shape Hash\tDescription\tDDSCAT Version Tag\t"
						"Frequency (GHz)\tDipole Spacing (um)\t"
						"M_real\tM_imag\tAeff (um)\tBetas\tThetas\tPhis\tNumber of Raw Orientations Available\t"
						//"V_Voronoi\tSA_Voronoi\tf_Voronoi\tV_Convex\tSA_Convex\tf_Convex\t"
						//"V_Ellipsoid_Max\tSA_Ellipsoid_Max\tEllipsoid_Max\t"
						//"V_Circum_Sphere\tSA_Circum_Sphere\tf_Circum_Sphere\t"
						"Qsca_iso\tQbk_iso\tQabs_iso\tQext_iso"<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<::rtmath::registry::IOhandler>
				export_tsv_ddori_iso_data
				(shared_ptr<::rtmath::registry::IOhandler> sh, shared_ptr<::rtmath::registry::IO_options> opts,
				const ::rtmath::ddscat::ddOutput *ddOut)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "isotropic_data") RTthrow debug::xUnimplementedFunction();
				std::string filename = opts->filename();
				std::string sDescrip = opts->getVal<std::string>("description","");
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_ddoutput_iso_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_ddoutput_iso_handle>(new tsv_ddoutput_iso_handle(filename.c_str(), iotype));
				else {
					if (sh->getId() != PLUGINID_VORO) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<tsv_ddoutput_iso_handle>(sh);
				}

				// Initial file creation handles writing the initial header.
				// So, just write the data.
				rotations rots;
				ddOut->avg->getRots(rots);
				/*
				if (!ddOut->stats)
				{
				if (ddOut->shape)
				ddOut->stats = boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats>(stats::shapeFileStats::genStats(ddOut->shape));
				}
				// Calculate scaled quantities
				double ds = ddOut->avg->dipoleSpacing();

				double Vvoro = ddOut->stats->SVoronoi_hull.V * pow(ds,3.);
				double Svoro = ddOut->stats->SVoronoi_hull.SA * pow(ds,2.);
				double fvoro = ddOut->stats->SVoronoi_hull.f;

				double Vconv = ddOut->stats->Sconvex_hull.V * pow(ds,3.);
				double Sconv = ddOut->stats->Sconvex_hull.SA * pow(ds,2.);
				double fconv = ddOut->stats->Sconvex_hull.f;

				double Vellm = ddOut->stats->Sellipsoid_max.V * pow(ds,3.);
				double Sellm = ddOut->stats->Sellipsoid_max.SA * pow(ds,2.);
				double fellm = ddOut->stats->Sellipsoid_max.f;

				double Vcirc = ddOut->stats->Scircum_sphere.V * pow(ds,3.);
				double Scirc = ddOut->stats->Scircum_sphere.SA * pow(ds,2.);
				double fcirc = ddOut->stats->Scircum_sphere.f;
				*/

				if (!sDescrip.size()) sDescrip=ddOut->description;

				(*(h->file.get())) << ddOut->shapeHash.lower << "\t" << sDescrip << "\t"
					<< ddOut->ddvertag << "\t"
					<< ddOut->avg->freq() << "\t" << ddOut->avg->dipoleSpacing() << "\t"
					<< ddOut->avg->getM().real() << "\t" << ddOut->avg->getM().imag() << "\t"
					<< ddOut->avg->aeff() << "\t" << rots.bN() << "\t" << rots.tN() << "\t" << rots.pN() << "\t"
					<< ddOut->scas.size() << "\t" // << ds << "\t"
					//<< Vvoro << "\t" << Svoro << "\t" << fvoro << "\t" 
					//<< Vconv << "\t" << Sconv << "\t" << fconv << "\t" 
					//<< Vellm << "\t" << Sellm << "\t" << fellm << "\t" 
					//<< Vcirc << "\t" << Scirc << "\t" << fcirc << "\t" 
					<< ddOut->avg->getStatEntry(stat_entries::QSCAM) << "\t"
					<< ddOut->avg->getStatEntry(stat_entries::QBKM) << "\t"
					<< ddOut->avg->getStatEntry(stat_entries::QABSM) << "\t"
					<< ddOut->avg->getStatEntry(stat_entries::QEXTM) << std::endl;
				;

				return h; // Pass back the handle
			}

		}
	}
}
