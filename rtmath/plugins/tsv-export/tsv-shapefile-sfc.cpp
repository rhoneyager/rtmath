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
					(*file.get()) << "Hash\tPoint\t"
						<< "X\tY\tZ\tNormed X\tNormed Y\tNormed Z\t"
						<< "Radius\tNormed Radius\t"
						<< "Index\tScaled Index"
						<< std::endl;
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

				auto hash = s->hash();
				auto csq = (s->latticePtsNorm.cwiseProduct(s->latticePtsNorm));
				Eigen::MatrixXf csqs(s->latticePtsNorm.rows(), 1);
				csqs = csq.block(0,0,csq.rows(),1) + csq.block(0,1,csq.rows(),1)
					+ csq.block(0,2,csq.rows(),1);
				// max_radius acts as a scaling factor.
				// For the radius calculation, all of the points are first
				// relocated according to the true mean. Sould be the center of
				// mass for a one-substance object.
				double max_radius = ::std::sqrt(csqs.maxCoeff());
				// For convolution operations, can rescale that dielectric
				// value (which means number of neighbors) by a factor.
				// The factor represents the convolution volume.
				double fScale = opts->getVal<double>("dielScalingFactor", 1.);

				// Can easily use latticePtsNorm to determine normed extent in X, Y and Z
				Eigen::Array<float, 1, 3> mins, maxs;
				mins(0,0) = s->latticePtsNorm.block(0,0,s->numPoints,1).minCoeff();
				mins(0,1) = s->latticePtsNorm.block(0,1,s->numPoints,1).minCoeff();
				mins(0,2) = s->latticePtsNorm.block(0,2,s->numPoints,1).minCoeff();
				maxs(0,0) = s->latticePtsNorm.block(0,0,s->numPoints,1).maxCoeff();
				maxs(0,1) = s->latticePtsNorm.block(0,1,s->numPoints,1).maxCoeff();
				maxs(0,2) = s->latticePtsNorm.block(0,2,s->numPoints,1).maxCoeff();
				//(*(h->file.get())) << s->numPoints << std::endl;

				std::vector<long> oi(s->numPoints * 7);

				for (size_t j = 0; j < s->numPoints; j++)
				{
					long point = s->latticeIndex(j);
					auto it = s->latticePts.block<1, 3>(j, 0);
					auto ot = s->latticePtsRi.block<1, 3>(j, 0);
					auto nt = s->latticePtsNorm.array().block<1, 3>(j, 0);
					Eigen::Array<float, Eigen::Dynamic, 3> off_min = nt - mins;
					Eigen::Array<float, Eigen::Dynamic, 3> scaled_min_a = off_min / (maxs - mins);
					auto cradsq = (nt).cwiseProduct(nt);
					double radsq = cradsq.sum();
					double rad = ::std::sqrt(radsq);
					double normrad = rad / max_radius;
					(*(h->file.get())) << hash.lower << "\t"
						<< j << "\t"
						<< (it)(0) << "\t" << (it)(1) <<
						"\t" << (it)(2) << "\t" 
						<< (scaled_min_a)(0) - 0.5 << "\t"
						<< (scaled_min_a)(1) - 0.5 << "\t"
						<< (scaled_min_a)(2) - 0.5 << "\t"
						<< rad << "\t" << normrad << "\t"
						<< (ot)(0) << "\t"
						<< ((double) (ot)(0)) / ((double) fScale) << std::endl;
				}
				//	<< s->standardD << "\t" << s->numPoints

				return h; // Pass back the handle
			}

		}
	}
}
