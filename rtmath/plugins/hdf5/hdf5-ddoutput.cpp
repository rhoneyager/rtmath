/// \brief Provides hdf5 file IO
#define _SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddScattMatrix.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include <hdf5.h>
#include <H5Cpp.h>
#include "plugin-hdf5.h"

#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"

#include "cmake-settings.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			void write_hdf5_ddPar(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::ddPar *r);
			bool read_hdf5_ddPar(std::shared_ptr<H5::Group> grpPar, 
				boost::shared_ptr<rtmath::ddscat::ddPar> &r);

			/// \param base is the base (./Runs) to write the subgroups to.
			std::shared_ptr<H5::Group> write_hdf5_ddOutput(std::shared_ptr<H5::Group> base, 
				std::shared_ptr<rtmath::registry::IO_options> opts, 
				const rtmath::ddscat::ddOutput *s)
			{
				using std::string;
				using std::shared_ptr;
				using namespace H5;

				bool writeFML = opts->getVal<bool>("writeFML", true);

				// Pick a unique name matching frequency, aeff and temperature (refractive index)
				/// \todo Pick a better naming function for hdf5-internal runs
				shared_ptr<Group> gRun(new Group(base->createGroup(s->genNameSmall())));


				addAttr<string, Group>(gRun, "Description", s->description);
				addAttr<double, Group>(gRun, "Frequency", s->freq);
				addAttr<double, Group>(gRun, "aeff", s->aeff);
				addAttr<double, Group>(gRun, "Temperature", s->temp);

				// Refractive indices table
				/*
				Eigen::MatrixXf refrs((int) s->ms.size(), 2);
				for (size_t i=0; i < s->ms.size(); ++i)
				{
					refrs(i,0) = (float) s->ms[i].real();
					refrs(i,1) = (float) s->ms[i].imag();
				}
				addDatasetEigen(gRun, "Refractive_Indices", refrs);
				*/
				// Source file paths

				// Tags

				// DDSCAT run verion tag
				addAttr<string, Group>(gRun, "DDSCAT_Version_Tag", s->ddvertag);

				// Enable compression
				hsize_t chunk_dimsOri[2] = { (hsize_t)s->oridata_d.rows(), 1 };
				hsize_t chunk_dimsFML[2] = { (hsize_t) s->fmldata->rows(), 
					rtmath::ddscat::ddOutput::fmlColDefs::NUM_FMLCOLDEFS };
				auto plistOri = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistOri->setChunk(2, chunk_dimsOri);
				auto plistFML = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistFML->setChunk(2, chunk_dimsFML);
				/* // SZIP compression disabled for now
#if COMPRESS_SZIP
				unsigned szip_options_mask = H5_SZIP_NN_OPTION_MASK;
				unsigned szip_pixels_per_block = 16;
				plistOri->setSzip(szip_options_mask, szip_pixels_per_block);
				plistFML->setSzip(szip_options_mask, szip_pixels_per_block);
				*/
#if COMPRESS_ZLIB
				plistOri->setDeflate(6);
				plistFML->setDeflate(6);
#endif

				addDatasetEigen(gRun, "Cross_Sections_d", (s->oridata_d), plistOri);
				addDatasetEigen(gRun, "Cross_Sections_i", (s->oridata_i), plistOri);
				//addDatasetEigen(gRun, "Cross_Sections_s", (s->oridata_s)); // , plistOri);
				//if (s->avg)
				//	addDatasetEigen(gRun, "Isotropic_Cross_Sections", *(s->avgoridata));
				if (writeFML)
					addDatasetEigen(gRun, "FML_Data", *(s->fmldata), plistFML);
				//addDatasetEigen(gRun, "Scattering_Data", s->scadata);

				
				// Shapefile link
				//addAttr<string,Group>(gRun, "Shapehash_full", s->shapeHash.string());
				addAttr<uint64_t,Group>(gRun, "Shapehash_lower", s->shapeHash.lower);
				addAttr<uint64_t,Group>(gRun, "Shapehash_upper", s->shapeHash.upper);

				// If a shapefile is written to this file, make a symlink
				std::string pShape;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->shapeHash.string() << "/Shape";
					pShape = o.str();
				}
				gRun->link(H5L_TYPE_SOFT, pShape, "Shape");


				// If stats are written to this file, make a symlink
				std::string pStats;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->shapeHash.string() << "/Stats";
					pShape = o.str();
				}
				gRun->link(H5L_TYPE_SOFT, pShape, "Stats");
				// Insert stats information given known dipole spacing


				// ddscat.par file
				write_hdf5_ddPar(
					shared_ptr<Group>(new Group(gRun->createGroup("par"))), 
					s->parfile.get());

				// Testing ddscat.par read...
				//boost::shared_ptr<rtmath::ddscat::ddPar> stest(new rtmath::ddscat::ddPar);
				//shared_ptr<Group> grpPar = openGroup(gRun, "par");
				//read_hdf5_ddPar(grpPar, stest);
				//stest->write(std::cout);


				return gRun;
			}
		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;
		
		template<>
		shared_ptr<IOhandler> 
			write_file_type_multi<rtmath::ddscat::ddOutput>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts, 
			const rtmath::ddscat::ddOutput *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename.c_str(), iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<hdf5_handle>(sh);
			}

			// Check for the existence of the appropriate:
			// Group "Hashed"
			shared_ptr<Group> grpHashes = openOrCreateGroup(h->file, "Hashed");
			// Group "Hashed"/shp->hash
			shared_ptr<Group> grpHash = openOrCreateGroup(grpHashes, s->shape->hash().string().c_str());
			// Group "Hashed"/shp->hash/"Shape". If it exists, overwrite it. There should be no hard links here.
			/// \note The unlink operation does not really free the space..... Should warn the user.
			shared_ptr<Group> gRuns = openOrCreateGroup(grpHash, "Runs");

			//std::string aeffid("aeff-");
			//aeffid.append(boost::lexical_cast<std::string>( (float) s->aeff ));
			//shared_ptr<Group> gRunsAeff = openOrCreateGroup(gRuns, aeffid.c_str());

			//if (groupExists(grpRuns, "Stats")) return h; //grpHash->unlink("Stats");

			/// \todo Modify to also support external symlinks
			shared_ptr<Group> base = write_hdf5_ddOutput(gRuns, opts, s);

			bool writeShape = opts->getVal("writeShapes", true);
			if (writeShape)
				write_file_type_multi<rtmath::ddscat::shapefile::shapefile>
					(h, opts, s->shape.get());

			//shared_ptr<Group> newstatsbase = write_hdf5_statsrawdata(grpHash, s);
			//shared_ptr<Group> newshapebase = write_hdf5_shaperawdata(grpHash, s->_shp.get());

			return h; // Pass back the handle
		}

		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<rtmath::ddscat::ddOutput>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			rtmath::ddscat::ddOutput *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename.c_str(), iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<hdf5_handle>(sh);
			}

			/* The reader function will attempt to load ddOutput files that match the opts 
			 * search descriptions.
			 * opts can search:
			 * - by hash
			 * - by effective radius
			 * - random particle
			 * - by temperature
			 * - by frequency
			 **/
			/// \todo Implement opts searching features


			return h;
		}
	}
}
