/// \brief Provides hdf5 file IO
#define _SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4251 ) // warning C4251: dll-interface
#pragma warning( disable : 4506 ) // warning C4251: spurious error (no definition for inline function) from boost/serialization/singleton.hpp

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

			bool read_hdf5_ddOutput(std::shared_ptr<H5::Group> base, std::shared_ptr<registry::IO_options> opts,
				rtmath::ddscat::ddOutput *r);

			/// \param base is the base (./Runs) to write the subgroups to.
			std::shared_ptr<H5::Group> write_hdf5_ddOutput(std::shared_ptr<H5::Group> base, 
				std::shared_ptr<rtmath::registry::IO_options> opts, 
				const rtmath::ddscat::ddOutput *s)
			{
				using std::string;
				using std::shared_ptr;
				using namespace H5;

				bool writeORI = opts->getVal<bool>("writeORI", true);
				bool writeFML = opts->getVal<bool>("writeFML", true);
				bool writeSHP = opts->getVal<bool>("writeSHP", true);
				bool writeAVG = opts->getVal<bool>("writeAVG", true);

				// Pick a unique name matching frequency, aeff and temperature (refractive index)
				/// \todo Pick a better naming function for hdf5-internal runs
				shared_ptr<Group> gRun(new Group(base->createGroup(s->genNameSmall())));


				addAttr<string, Group>(gRun, "Description", s->description);
				addAttr<string, Group>(gRun, "ingest_timestamp", s->ingest_timestamp);
				addAttr<string, Group>(gRun, "ingest_hostname", s->ingest_hostname);
				addAttr<string, Group>(gRun, "ingest_username", s->ingest_username); // Not all ingests have this...
				addAttr<int, Group>(gRun, "ingest_rtmath_version", s->ingest_rtmath_version);
				addAttr<string, Group>(gRun, "hostname", s->hostname);
				addAttr<double, Group>(gRun, "Frequency", s->freq);
				addAttr<double, Group>(gRun, "aeff", s->aeff);
				addAttr<double, Group>(gRun, "Temperature", s->temp);


				addAttr<size_t, Group>(gRun, "DDSCAT_Version_Num", s->s.version);
				addAttr<size_t, Group>(gRun, "Num_Dipoles", s->s.num_dipoles);
				addAttr<size_t, Group>(gRun, "Num_Avgs_cos", s->s.navg);


				addAttr<string, Group>(gRun, "target", s->s.target);
				//addAttr<string, Group>(gRun, "ddameth", s->ddameth);
				//addAttr<string, Group>(gRun, "ccgmeth", s->ccgmeth);
				//addAttr<string, Group>(gRun, "hdr_shape", s->hdr_shape);

				addAttrArray<double, Group>(gRun, "mins", s->s.mins.data(), 1, 3);
				addAttrArray<double, Group>(gRun, "maxs", s->s.maxs.data(), 1, 3);
				addAttrArray<double, Group>(gRun, "TA1TF", s->s.TA1TF.data(), 1, 3);
				addAttrArray<double, Group>(gRun, "TA2TF", s->s.TA2TF.data(), 1, 3);
				addAttrArray<double, Group>(gRun, "LFK", s->s.LFK.data(), 1, 3);

				addAttrComplex<std::complex<double>, Group>
					(gRun, "IPV1LF", s->s.IPV1LF.data(), 1, 3);
				addAttrComplex<std::complex<double>, Group>
					(gRun, "IPV2LF", s->s.IPV2LF.data(), 1, 3);


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
				{
					std::vector<string> srcs(s->sources.begin(), s->sources.end());
					std::vector<const char*> csrcs(srcs.size());
					for (size_t i = 0; i<srcs.size(); ++i)
						csrcs[i] = srcs[i].c_str();
					addAttr<size_t, Group>(gRun, "Num_Source_Paths", srcs.size());
					addDatasetArray<const char*, Group>(gRun, "Sources", csrcs.size(), 1, csrcs.data());
				}
				// Tags
				{
					addAttr<size_t, Group>(gRun, "Num_Tags", s->tags.size());
					
					const size_t nTagCols = 2;
					typedef std::array<const char*, nTagCols> strdata;
					std::vector<strdata> sdata(s->tags.size());
					size_t i = 0;
					for (const auto &t : s->tags)
					{
						sdata.at(i).at(0) = t.first.c_str();
						sdata.at(i).at(1) = t.second.c_str();
						++i;
					}

					hsize_t dim[1] = { static_cast<hsize_t>(s->tags.size()) };
					DataSpace space(1, dim);
					// May have to cast array to a private structure
					H5::StrType strtype(0, H5T_VARIABLE);
					CompType stringTableType(sizeof(strdata));
					stringTableType.insertMember("Key", ARRAYOFFSET(strdata, 0), strtype);
					stringTableType.insertMember("Value", ARRAYOFFSET(strdata, 1), strtype);
					std::shared_ptr<DataSet> sdataset(new DataSet(gRun->createDataSet(
						"Tags", stringTableType, space)));
					sdataset->write(sdata.data(), stringTableType);
				}

				// DDSCAT run verion tag
				addAttr<string, Group>(gRun, "DDSCAT_Version_Tag", s->ddvertag);

				if (writeORI && s->oridata_d.rows())
				{
					auto csd = addDatasetEigen(gRun, "Cross_Sections", (s->oridata_d), make_plist(s->oridata_d.rows(), 1););
					addColNames(csd, rtmath::ddscat::ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES, 
						rtmath::ddscat::ddOutput::stat_entries::stringify);
				}

				if (writeAVG)
				{
					if (s->avg(0)) // Only write if there is something to write
					{
						auto avg = addDatasetEigen(gRun, "Average_Results", (s->avg), make_plist(1, rtmath::ddscat::ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES));
						addColNames(avg, rtmath::ddscat::ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES,
							rtmath::ddscat::ddOutput::stat_entries::stringify);
					}
				}

				/*
				// String table can't be compressed...
				// This will be written as a compound datatype
				{
					typedef std::array<const char*, rtmath::ddscat::ddOutput::stat_entries::NUM_STAT_ENTRIES_STRINGS> strdata;
					std::vector<strdata> sdata(s->oridata_s.size());
					for (size_t i = 0; i < s->oridata_s.size(); ++i)
						for (size_t j = 0; j < rtmath::ddscat::ddOutput::stat_entries::NUM_STAT_ENTRIES_STRINGS; ++j)
						{
						sdata.at(i).at(j) = s->oridata_s.at(i).at(j).c_str();
						}


					hsize_t dim[1] = { static_cast<hsize_t>(s->oridata_s.size()) };
					DataSpace space(1, dim);
					// May have to cast array to a private structure
					H5::StrType strtype(0, H5T_VARIABLE);

					CompType stringTableType(sizeof(strdata));
					//#define HOFFSETORIG(TYPE, MEMBER) ((size_t) &((TYPE *)0)-> MEMBER)
					
					if (0) {
					std::cerr << ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::TARGET) << " "
						<< ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::DDAMETH) << " "
						<< ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::CCGMETH) << " "
						<< ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::SHAPE) << " "
						<< sizeof(strdata) << " " 
						<< (size_t) &(sdata[0]) - (size_t) sdata[0].data() << " " 
						<< (size_t) &(sdata[1]) - (size_t)sdata[1].data() << " " 
						<< (size_t) &(sdata[0].at(1)) - (size_t)&(sdata[0]) << " "
						<< (size_t)&(sdata[0].at(2)) - (size_t)&(sdata[0]) << " "
						<< (size_t)&(sdata[0].at(3)) - (size_t)&(sdata[0]) << " "
						<< std::endl;
					}

					stringTableType.insertMember("TARGET", ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::TARGET), strtype);
					stringTableType.insertMember("DDAMETH", ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::DDAMETH), strtype);
					stringTableType.insertMember("CCGMETH", ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::CCGMETH), strtype);
					stringTableType.insertMember("SHAPE", ARRAYOFFSET(strdata, rtmath::ddscat::ddOutput::stat_entries::SHAPE), strtype);
					std::shared_ptr<DataSet> sdataset(new DataSet(gRun->createDataSet(
						"Cross_Sections_s", stringTableType, space)));
					sdataset->write(sdata.data(), stringTableType);
				}
				*/

				//addDatasetEigen(gRun, "Cross_Sections_s", (s->oridata_s)); // , plistOri);
				//if (s->avg)
				//	addDatasetEigen(gRun, "Isotropic_Cross_Sections", *(s->avgoridata));
				if (writeFML && s->fmldata->rows())
				{
					auto ft = addDatasetEigen(gRun, "FML_Data", *(s->fmldata), make_plist(s->fmldata->rows(),
						rtmath::ddscat::ddOutput::fmlColDefs::NUM_FMLCOLDEFS));
					addColNames(ft, rtmath::ddscat::ddOutput::fmlColDefs::NUM_FMLCOLDEFS,
						rtmath::ddscat::ddOutput::fmlColDefs::stringify);
				}
				//addDatasetEigen(gRun, "Scattering_Data", s->scadata);

				
				// Shapefile link
				//addAttr<string,Group>(gRun, "Shapehash_full", s->shapeHash.string());
				addAttr<uint64_t,Group>(gRun, "Shapehash_lower", s->shapeHash.lower);
				addAttr<uint64_t,Group>(gRun, "Shapehash_upper", s->shapeHash.upper);
				addAttr<uint64_t, Group>(gRun, "ParsedShapehash_lower", s->parsedShapeHash.lower);
				addAttr<uint64_t, Group>(gRun, "ParsedShapehash_upper", s->parsedShapeHash.upper);


				// If a shapefile is written to this file, make a symlink
				std::string pShape;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->shapeHash.string() << "/Shape";
					pShape = o.str();
				}
				if (writeSHP)
					gRun->link(H5L_TYPE_SOFT, pShape, "Shape");

				// ddscat.par file
				write_hdf5_ddPar(
					shared_ptr<Group>(new Group(gRun->createGroup("par"))), 
					s->parfile.get());

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

			bool writeShape = opts->getVal<bool>("writeSHP", true);
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
			std::string hash = opts->getVal<std::string>("hash");
			std::string key = opts->getVal<std::string>("key");
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

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (!grpHashes) RTthrow debug::xMissingFile(key.c_str());
			shared_ptr<Group> grpHash = openGroup(grpHashes, hash.c_str());
			if (!grpHash) RTthrow debug::xMissingFile(hash.c_str());
			shared_ptr<Group> grpRuns = openGroup(h->file, "Runs");
			if (!grpRuns) RTthrow debug::xMissingFile(key.c_str());
			shared_ptr<Group> grpRun = openGroup(grpRuns, key.c_str());
			if (!grpRun) RTthrow debug::xMissingFile(key.c_str());
			read_hdf5_ddOutput(grpRun, opts, s);

			return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<rtmath::ddscat::ddOutput>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::ddscat::ddOutput> > &s)
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
			}
			else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<hdf5_handle>(sh);
			}

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (grpHashes)
			{
				hsize_t sz = grpHashes->getNumObjs();
				//s.reserve(s.size() + sz);
				for (hsize_t i = 0; i < sz; ++i)
				{
					std::string hname = grpHashes->getObjnameByIdx(i);
					H5G_obj_t t = grpHashes->getObjTypeByIdx(i);
					if (t != H5G_obj_t::H5G_GROUP) continue;
					if (key.size() && key != hname) continue;

					shared_ptr<Group> grpHash = openGroup(grpHashes, hname.c_str());
					if (!grpHash) continue; // Should never happen
					shared_ptr<Group> grpRuns = openGroup(grpHash, "Runs");
					if (!grpRuns) continue;

					hsize_t rz = grpRuns->getNumObjs();
					for (hsize_t i = 0; i < rz; ++i)
					{
						std::string runname = grpRuns->getObjnameByIdx(i);
						H5G_obj_t t = grpRuns->getObjTypeByIdx(i);
						if (t != H5G_obj_t::H5G_GROUP) continue;
						shared_ptr<H5::Group> grpRun = openGroup(grpRuns, runname.c_str());
						if (!grpRun) continue;

						boost::shared_ptr<rtmath::ddscat::ddOutput>
							run(new rtmath::ddscat::ddOutput);
						read_hdf5_ddOutput(grpRun, opts, run.get());
						s.push_back(run);
					}
				}
			}

			return h;
		}
	}
}
