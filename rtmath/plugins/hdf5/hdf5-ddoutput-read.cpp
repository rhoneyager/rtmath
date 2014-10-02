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
#include <vector>

#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddScattMatrix.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include <hdf5.h>
#include <H5Cpp.h>
#include "plugin-hdf5.h"
#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			template<class T, class Container>
			void readAttrSet(std::shared_ptr<H5::Group> grpPar, const char* name,
				boost::shared_ptr<rtmath::ddscat::ddPar> r, void (rtmath::ddscat::ddOutput::* f) (const T&))
			{
				T val;
				readAttr<T, Container>(grpPar, name, val);
				(*r.*f)(val);
			}

			template<class T, class Container>
			void readAttrSet(std::shared_ptr<H5::Group> grpPar, const char* name,
				boost::shared_ptr<rtmath::ddscat::ddPar> r, void (rtmath::ddscat::ddOutput::* f) (T))
			{
				T val;
				readAttr<T, Container>(grpPar, name, val);
				(*r.*f)(val);
			}

			bool read_hdf5_ddPar(std::shared_ptr<H5::Group> grpPar,
				boost::shared_ptr<rtmath::ddscat::ddPar > r);
			bool read_hdf5_shaperawdata(std::shared_ptr<H5::Group> base, 
				boost::shared_ptr<rtmath::ddscat::shapefile::shapefile > shp);
			bool read_hdf5_ddOutput(std::shared_ptr<H5::Group> base, std::shared_ptr<registry::IO_options> opts, 
				boost::shared_ptr<rtmath::ddscat::ddOutput > r)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace ddscat;
				
				readAttr<string, Group>(base, "Description", r->description);
				readAttr<double, Group>(base, "Frequency", r->freq);
				readAttr<double, Group>(base, "aeff", r->aeff);
				readAttr<double, Group>(base, "Temperature", r->temp);
				readAttr<string, Group>(base, "ingest_timestamp", r->ingest_timestamp);
				readAttr<string, Group>(base, "ingest_hostname", r->ingest_hostname);
				if (attrExists(base, "ingest_username"))
					readAttr<string, Group>(base, "ingest_username", r->ingest_username);
				//if (attrExists(base, "run_uuid"))
				//	readAttr<string, Group>(base, "run_uuid", r->runuuid);
				readAttr<int, Group>(base, "ingest_rtmath_version", r->ingest_rtmath_version);
				readAttr<string, Group>(base, "hostname", r->hostname);

				readAttr<size_t, Group>(base, "DDSCAT_Version_Num", r->s.version);
				readAttr<size_t, Group>(base, "Num_Dipoles", r->s.num_dipoles);
				readAttr<size_t, Group>(base, "Num_Avgs_cos", r->s.navg);
				readAttr<string, Group>(base, "target", r->s.target);

				readAttrArray<double, Group>(base, "mins", r->s.mins.data(), 1, 3);
				readAttrArray<double, Group>(base, "maxs", r->s.maxs.data(), 1, 3);
				readAttrArray<double, Group>(base, "TA1TF", r->s.TA1TF.data(), 1, 3);
				readAttrArray<double, Group>(base, "TA2TF", r->s.TA2TF.data(), 1, 3);
				readAttrArray<double, Group>(base, "LFK", r->s.LFK.data(), 1, 3);

				readAttrComplex<std::complex<double>, Group>
					(base, "IPV1LF", r->s.IPV1LF.data(), 3, 1);
				readAttrComplex<std::complex<double>, Group>
					(base, "IPV2LF", r->s.IPV2LF.data(), 3, 1);

				/*
				Eigen::MatrixXf refrs;
				readDatasetEigen<Eigen::MatrixXf, Group>(base, "Refractive_Indices", refrs);
				for (size_t i=0; i< (size_t) refrs.rows(); ++i)
					r->ms.push_back(std::complex<double>( refrs(i,0), refrs(i,1) ));
				*/

				// Source file paths
				{
					std::vector<size_t> dims;
					readDatasetDimensions<Group>(base, "Sources", dims);
					std::vector<const char*> csrcs(dims[0]);
					readDatasetArray<const char*, Group>(base, "Sources", csrcs.data());
					for (size_t i = 0; i<csrcs.size(); ++i)
						r->sources.insert(std::string(csrcs[i]));
					
				}

				// Tags
				{
					const size_t nTagCols = 2;
					typedef std::array<const char*, nTagCols> strdata;
					// Get size of the tags object
					size_t numTags = 0;
					readAttr<size_t, Group>(base, "Num_Tags", numTags);

					if (numTags)
					{
						std::vector<strdata> sdata(numTags);

						hsize_t dim[1] = { static_cast<hsize_t>(numTags) };
						DataSpace space(1, dim);
						// May have to cast array to a private structure
						H5::StrType strtype(0, H5T_VARIABLE);
						CompType stringTableType(sizeof(strdata));
						stringTableType.insertMember("Key", ARRAYOFFSET(strdata, 0), strtype);
						stringTableType.insertMember("Value", ARRAYOFFSET(strdata, 1), strtype);

						std::shared_ptr<DataSet> sdataset(new DataSet(base->openDataSet("Tags")));
						sdataset->read(sdata.data(), stringTableType);

						for (const auto &t : sdata)
						{
							r->tags.insert(std::pair<std::string, std::string>(
								std::string(t.at(0)), std::string(t.at(1))));
						}
					}
				}

				// DDSCAT run version tag
				if (attrExists(base, "DDA_Version_Tag") )
					readAttr<string, Group>(base, "DDA_Version_Tag", r->ddvertag);
				else if (attrExists(base, "DDSCAT_Version_Tag") )
					readAttr<string, Group>(base, "DDSCAT_Version_Tag", r->ddvertag);

				readAttr<uint64_t, Group>(base, "Shapehash_lower", r->shapeHash.lower);
				readAttr<uint64_t, Group>(base, "Shapehash_upper", r->shapeHash.upper);
				readAttr<uint64_t, Group>(base, "ParsedShapehash_lower", r->parsedShapeHash.lower);
				readAttr<uint64_t, Group>(base, "ParsedShapehash_upper", r->parsedShapeHash.upper);

				if (attrExists(base, "runhash_lower")) // Not all have this
				{
					readAttr<uint64_t, Group>(base, "runhash_lower", r->_runhash.lower);
					readAttr<uint64_t, Group>(base, "runhash_upper", r->_runhash.upper);
				}

				bool readSHP = opts->getVal<bool>("readSHP", false);
				bool readORI = opts->getVal<bool>("readORI", true);
				bool readFML = opts->getVal<bool>("readFML", true);
				bool readAVG = opts->getVal<bool>("readAVG", true);

				if (readORI && datasetExists(base, "Cross_Sections"))
					readDatasetEigen(base, "Cross_Sections", (r->oridata_d));
				if (readFML && datasetExists(base, "FML_Data"))
					readDatasetEigen(base, "FML_Data", *(r->fmldata));
				if (readAVG && datasetExists(base, "Average_Results"))
				{
					auto tbl = readDatasetEigen(base, "Average_Results", (r->avgdata.avg));
					r->avgdata.hasAvg = true;
					readAttr(tbl, "beta_min", r->avgdata.beta_min);
					readAttr(tbl, "beta_max", r->avgdata.beta_max);
					readAttr(tbl, "beta_n", r->avgdata.beta_n);
					readAttr(tbl, "theta_min", r->avgdata.theta_min);
					readAttr(tbl, "theta_max", r->avgdata.theta_max);
					readAttr(tbl, "theta_n", r->avgdata.theta_n);
					readAttr(tbl, "phi_min", r->avgdata.phi_min);
					readAttr(tbl, "phi_max", r->avgdata.phi_max);
					readAttr(tbl, "phi_n", r->avgdata.phi_n);
				}
				if (readSHP) {
					// Try these, in order:
					if (rtmath::ddscat::shapefile::shapefile::isHashStored(r->shapeHash)) {
						r->shape = rtmath::ddscat::shapefile::shapefile::loadHash(r->shapeHash);
					} else if (symLinkExists(base, "Shape").second) {
						// Read the shape and store a pointer within the ddOutput object.
						// Also, register this shape with the shapefile routines.
						shared_ptr<Group> grpShape = openGroup(base, "Shape");
						boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> shp = 
							rtmath::ddscat::shapefile::shapefile::generate();
						read_hdf5_shaperawdata(grpShape, shp);
						shp->registerHash();
						r->shape = shp;
					} else {
						r->shape = rtmath::ddscat::shapefile::shapefile::loadHash(r->shapeHash);
					}
				}
				
				// Do, however, read the ddscat.par file, since some of these values are useful when 
				// interpreting the ddscat run.
				r->parfile = ddPar::generate(); //boost::shared_ptr<ddPar>(new ddPar);
				read_hdf5_ddPar(openGroup(base, "par"), r->parfile);
				//r->doImport();
				return true;
			}
		}
	}

}
