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
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
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
				rtmath::ddscat::ddPar *r);

			bool read_hdf5_ddOutput(std::shared_ptr<H5::Group> base, std::shared_ptr<registry::IO_options> opts, 
				boost::shared_ptr<rtmath::ddscat::ddOutput> &r)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace ddscat;
				
				readAttr<string, Group>(base, "Description", r->description);
				readAttr<double, Group>(base, "Frequency", r->freq);
				readAttr<double, Group>(base, "aeff", r->aeff);
				readAttr<double, Group>(base, "Temperature", r->temp);

				Eigen::MatrixXf refrs;
				readDatasetEigen<Eigen::MatrixXf, Group>(base, "Refractive_Indices", refrs);
				for (size_t i=0; i< (size_t) refrs.rows(); ++i)
					r->ms.push_back(std::complex<double>( refrs(i,0), refrs(i,1) ));

				// Source file paths

				// Tags

				// DDSCAT run version tag
				readAttr<string, Group>(base, "DDSCAT_Version_Tag", r->ddvertag);


				readDatasetEigen(base, "Cross_Sections", *(r->oridata));
				if (datasetExists(base, "Isotropic_Cross_Sections"))
					readDatasetEigen(base, "Isotropic_Cross_Sections", *(r->avgoridata));

				bool readFML = opts->getVal<bool>("readFML", true);
				if (readFML && datasetExists(base, "FML_Data"))
				{
					readDatasetEigen(base, "FML_Data", *(r->fmldata));
				}
				//readDatasetEigen(base, "Scattering_Data", r->scadata);

				readAttr<uint64_t, Group>(base, "Shapehash_lower", r->shapeHash.lower);
				readAttr<uint64_t, Group>(base, "Shapehash_upper", r->shapeHash.upper);

				// The shapefiles are loaded in a separate bit of code, and they have their own search 
				// directory. The same applies to shape stats. As such, don't read the symlinks in this 
				// iteration of the code.

				// Do, however, read the ddscat.par file, since some of these values are useful when 
				// interpreting the ddscat run.
				r->parfile = boost::shared_ptr<ddPar>(new ddPar);
				read_hdf5_ddPar(openGroup(base, "par"), r->parfile.get());
				r->doImport();
				return true;
			}
		}
	}

}
