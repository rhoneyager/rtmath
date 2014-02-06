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
#include <hdf5.h>
#include <H5Cpp.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddScattMatrix.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {
			
			std::shared_ptr<H5::Group> write_hdf5_ddScattMatrix(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::ddScattMatrix *v)
			{
				using std::shared_ptr;
				using namespace H5;
				using namespace rtmath::ddscat;

				std::string rotid;
				{
					std::ostringstream o;
					o << v->theta() << "," << v->phi() << "," << v->thetan() << "," << v->phin();
					rotid = o.str();
				}

				shared_ptr<Group> gv(new Group(base->createGroup(rotid)));
				
				addAttr<double, Group>(gv, "Pol", v->pol());
				addAttr<double, Group>(gv, "Pol_Linear", v->polLin());
				addAttr<double, Group>(gv, "Frequency", v->freq());
				addAttr<double, Group>(gv, "theta", v->theta());
				addAttr<double, Group>(gv, "thetan", v->thetan());
				addAttr<double, Group>(gv, "phi", v->phi());
				addAttr<double, Group>(gv, "phin", v->phin());

				addDatasetEigen<ddScattMatrix::PnnType, Group>(gv, "Mueller", v->mueller());

				if (v->id() == scattMatrixType::F)
				{
					const ddScattMatrixF *vv = dynamic_cast<const ddScattMatrixF*>(v);
					/// \todo Add complex double support
					addDatasetEigenComplexMethodA<ddScattMatrix::FType, Group>(gv, "F", vv->getF());
					addDatasetEigenComplexMethodA<ddScattMatrix::FType, Group>(gv, "S", vv->getS());
				}

				return gv;
			}

			std::shared_ptr<H5::Group> write_hdf5_ddOutputSingle(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::ddOutputSingle *r, bool inplace = false)
			{
				using std::shared_ptr;
				using namespace H5;

				std::string rotid;
				{
					std::ostringstream o;
					o << r->beta() << "," << r->theta() << "," << r->phi();
					rotid = o.str();
				}
				
				shared_ptr<Group> grpRot;
				if (!inplace)
					grpRot = shared_ptr<Group> (new Group(base->createGroup(rotid)));
				else grpRot = base;
				
				addAttr<double, Group>(grpRot, "Beta", r->beta());
				addAttr<double, Group>(grpRot, "Theta", r->theta());
				addAttr<double, Group>(grpRot, "Phi", r->phi());

				addAttr<double, Group>(grpRot, "Wavelength", r->wave());
				addAttr<double, Group>(grpRot, "Frequency", r->freq());
				addAttr<double, Group>(grpRot, "aeff", r->aeff());
				addAttr<double, Group>(grpRot, "Dipole_Spacing", r->dipoleSpacing());
				addAttr<size_t, Group>(grpRot, "Num_Dipoles", r->numDipoles());

				addAttr<size_t, Group>(grpRot, "version", r->version());

				/// \todo add complex double support
				//addAttr<std::complex<double>, Group>(grpRot, "Refractive_Index", r->getM());

				// Header Entries
				//shared_ptr<Group> headers(new Group(grpRot->createGroup("Headers")));
				{
					// Create a dataset from strings

				}

				// Stat table

				// Scattering matrices
				{
					shared_ptr<Group> g(new Group(grpRot->createGroup("Scattering_Matrices")));
					ddscat::ddOutputSingle::scattMatricesContainer c;
					r->getScattMatrices(c);
					for (const auto &mat : c)
						write_hdf5_ddScattMatrix(g, mat.get());
				}

				return grpRot;
			}

			/// \param base is the base (./Runs) to write the subgroups to.
			std::shared_ptr<H5::Group> write_hdf5_ddOutput(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::ddOutput *s)
			{
				using std::string;
				using std::shared_ptr;
				using namespace H5;

				// Pick a unique name matching frequency, aeff and temperature (refractive index)
				shared_ptr<Group> gRun(new Group(base->createGroup(s->genName())));


				addAttr<string, Group>(gRun, "Description", s->description);
				addAttr<double, Group>(gRun, "Frequency", s->freq);
				addAttr<double, Group>(gRun, "aeff", s->aeff);

				// Refractive indices table

				// Source file paths

				// Tags

				// DDSCAT run verion tag
				addAttr<string, Group>(gRun, "DDSCAT_Version_Tag", s->ddvertag);

				// Ensemble average results
				{
					shared_ptr<Group> gEns(new Group(gRun->createGroup("Ensemble")));
					write_hdf5_ddOutputSingle(gEns, s->avg.get(), true);
					shared_ptr<Group> gEnso(new Group(gRun->createGroup("Ensemble_Original")));
					write_hdf5_ddOutputSingle(gEnso, s->avg_original.get(), true);
				}
				auto writeGroup = [&](const char* grpname, 
					const std::set<boost::shared_ptr<rtmath::ddscat::ddOutputSingle> > &o)
				{
					shared_ptr<Group> g(new Group(gRun->createGroup(grpname)));
					for (const auto& f : o)
						write_hdf5_ddOutputSingle(g, f.get());
				};
				writeGroup("FML", s->fmls);
				writeGroup("SCA", s->scas);
				writeGroup("SCA_original", s->scas_original);
				

				// Weight table (assuming all scas and fmls are accounted for)

				// Stats link

				// Insert stats information given known dipole spacing

				// Shapefile link

				// ddscat.par file



				return gRun;
			}

			/// \todo Add in writing handlers for ddoutputsingle and ddscatmatrix

			std::shared_ptr<rtmath::registry::IOhandler> write_hdf5_multi_ddoutputs
				(std::shared_ptr<rtmath::registry::IOhandler> sh, 
				const char* filename, 
				const rtmath::ddscat::ddOutput *s, 
				const char* key, // Unused for this type of write
				rtmath::registry::IOhandler::IOtype iotype)
			{
				using std::shared_ptr;
				using namespace H5;
				Exception::dontPrint();
				std::shared_ptr<hdf5_handle> h;
				if (!sh)
				{
					// Access the hdf5 file
					h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename, iotype));
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
				shared_ptr<Group> gRuns = openOrCreateGroup(grpHashes, "Runs");

				//if (groupExists(grpRuns, "Stats")) return h; //grpHash->unlink("Stats");

				/// \todo Modify to also support external symlinks
				shared_ptr<Group> base = write_hdf5_ddOutput(gRuns, s);
				//shared_ptr<Group> newstatsbase = write_hdf5_statsrawdata(grpHash, s);
				//shared_ptr<Group> newshapebase = write_hdf5_shaperawdata(grpHash, s->_shp.get());

				return h; // Pass back the handle
			}

			/// Routine writes a full, isolated shapefile entry
			void write_hdf5_ddOutput(const char* filename,
				const rtmath::ddscat::ddOutput *s)
			{
				try {
					using std::string;
					using std::ofstream;
					using std::shared_ptr;
					using namespace H5;

					// Turn off the auto-printing when failure occurs so that we can
					// handle the errors appropriately
					Exception::dontPrint();

					shared_ptr<H5File> file(new H5File(filename, H5F_ACC_TRUNC ));
					shared_ptr<Group> grpHashes(new Group(file->createGroup("Hashed")));
					shared_ptr<Group> shpgroup(new Group(grpHashes->createGroup(s->shape->hash().string().c_str())));
					shared_ptr<Group> gRuns(new Group(shpgroup->createGroup("Runs")));
					shared_ptr<Group> base = write_hdf5_ddOutput(gRuns, s);
					//shared_ptr<Group> shapebase = write_hdf5_shaperawdata(shpgroup, s->_shp.get());


					//statsbase->link(H5L_TYPE_HARD, ".", "/Stats");
					//shapebase->link(H5L_TYPE_HARD, ".", "/Shape");
					//file->link(H5L_TYPE_SOFT, newbase->, "Shape");
				} catch (std::exception &e)
				{
					std::cerr << e.what() << "\n";
					throw e;
				}
			}

		}
	}
}
