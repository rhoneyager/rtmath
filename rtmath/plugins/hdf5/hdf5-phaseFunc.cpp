/// \brief Provides hdf5 file IO
#define _SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <string>
#include <array>
#include <iostream>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-hdf5.h"

// These go last, as they do odd stuff to definitions.
#include <hdf5.h>
#include <H5Cpp.h>
namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			template<class T, class U>
			void vectorize(const T& src, U &arr, size_t r)
			{
				// Passing in an _array_ of Eigen matrices. Take each matrix and copy the internal data into 
				// the output array.
				size_t i = 0;
				for (const auto &mat : src)
				{
					for (size_t row = 0; row < (size_t)mat.rows(); ++row)
						for (size_t col = 0; col < (size_t)mat.cols(); ++col)
						{
						arr(r, i) = mat(row, col);
						++i;
						}
				}
			}



			/// \param base is the base to write the subgroups to. From here, "./Stats" is the root of the routine's output.
			std::shared_ptr<H5::Group> write_hdf5_pfrawdata(std::shared_ptr<H5::Group> base,
				const boost::shared_ptr<const rtmath::phaseFuncs::pfRunSetContainer > s)
			{
				using std::shared_ptr;
				using namespace H5;

				shared_ptr<Group> statsraw = base; // (new Group(base->createGroup("Stats")));
				
				auto make_plist = [](size_t rows, size_t cols)
				{
					hsize_t chunk[2] = { (hsize_t)rows, (hsize_t)cols };
					auto plist = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
					plist->setChunk(2, chunk);
#if COMPRESS_ZLIB
					plist->setDeflate(6);
#endif
					return plist;
				};

				int fillvalue = -1;   /* Fill value for the dataset */
				DSetCreatPropList plist;
				plist.setFillValue(PredType::NATIVE_INT, &fillvalue);

				using std::string;
				addAttr<string, Group>(statsraw, "ingest_timestamp", s->ingest_timestamp);
				addAttr<string, Group>(statsraw, "ingest_hostname", s->ingest_hostname);
				addAttr<string, Group>(statsraw, "ingest_username", s->ingest_username);
				addAttr<int, Group>(statsraw, "ingest_rtmath_version", s->ingest_rtmath_version);
				addAttr<int, Group>(statsraw, "phaseFunc_version", s->phaseFunc_version);

				using std::vector;
				using namespace rtmath::phaseFuncs;
				vector<const char*> providers;
				vector<pf_class_registry::setup> setups;
				struct transInputParamsPartial
				{
					double aeff;
					// hdf5 enumtype aeff_version_type
					double m_real, m_im;
					// refractive index scaling method
					bool aeff_rescale;
					double vFrac;
					const char* ref;
					// hdf5 enumtype shape_type
					double eps;
					transInputParamsPartial() 
						: aeff(0), m_real(0), m_im(0), aeff_rescale(0), vFrac(0), ref(nullptr), eps(0) 
					{}
					transInputParamsPartial(const pf_class_registry::inputParamsPartial &p)
						: aeff(0), m_real(0), m_im(0), aeff_rescale(0), vFrac(0), ref(nullptr), eps(0)
					{

					}
				};
				vector<transInputParamsPartial> inputs;
				vector<pf_class_registry::cross_sections> css;

				providers.resize(s->runs.size());
				setups.resize(s->runs.size());
				inputs.resize(s->runs.size());
				css.resize(s->runs.size());

				for (size_t i = 0; i < s->runs.size(); ++i)
				{
					providers[i] = s->runs[i].providerName;
					setups[i] = s->runs[i].setup;
					inputs[i] = transInputParamsPartial(s->runs[i].i);
					css[i] = s->runs[i].cs;
				}

				const char* ippNames[] = { "aeff", "aeff_version_type", "m_real", "m_imag",
					"Refr_Index_Scaling_Meth", "aeff_rescale_flag", "vFrac", "FlakeRef",
					"shape_type", "ar" };

				hsize_t dim[1] = { s->runs.size() };
				DataSpace space(1, dim);
				CompType sTypeSetups(sizeof(pf_class_registry::setup));
				CompType sTypeInputs(sizeof(transInputParamsPartial));
				CompType sTypeCSS(sizeof(pf_class_registry::cross_sections));
				H5::StrType strtype(0, H5T_VARIABLE);

				sTypeSetups.insertMember("beta", HOFFSET(pf_class_registry::setup, beta), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("theta", HOFFSET(pf_class_registry::setup, theta), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("phi", HOFFSET(pf_class_registry::setup, phi), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("sTheta", HOFFSET(pf_class_registry::setup, sTheta), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("sTheta0", HOFFSET(pf_class_registry::setup, sTheta0), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("sPhi", HOFFSET(pf_class_registry::setup, sPhi), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("sPhi0", HOFFSET(pf_class_registry::setup, sPhi0), PredType::NATIVE_DOUBLE);
				sTypeSetups.insertMember("wavelength", HOFFSET(pf_class_registry::setup, wavelength), PredType::NATIVE_DOUBLE);

				sTypeCSS.insertMember("Qbk", HOFFSET(pf_class_registry::cross_sections, Qbk), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qext", HOFFSET(pf_class_registry::cross_sections, Qext), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qsca", HOFFSET(pf_class_registry::cross_sections, Qsca), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qabs", HOFFSET(pf_class_registry::cross_sections, Qabs), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("g", HOFFSET(pf_class_registry::cross_sections, g), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qbk_iso", HOFFSET(pf_class_registry::cross_sections, Qbk_iso), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qext_iso", HOFFSET(pf_class_registry::cross_sections, Qext_iso), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qsca_iso", HOFFSET(pf_class_registry::cross_sections, Qsca_iso), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("Qabs_iso", HOFFSET(pf_class_registry::cross_sections, Qabs_iso), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("g_iso", HOFFSET(pf_class_registry::cross_sections, g_iso), PredType::NATIVE_DOUBLE);
				sTypeCSS.insertMember("valid", HOFFSET(pf_class_registry::cross_sections, valid), PredType::NATIVE_HBOOL);


				std::shared_ptr<DataSet> gV(new DataSet(statsraw->createDataSet("Volumetric", sType, space)));
				gV->write(data.data(), sType);


				// Rotations
				{
					shared_ptr<Group> grpRotations(new Group(statsraw->createGroup("Rotations")));

					using namespace std;
					using namespace rtmath::ddscat::stats;
					Eigen::Matrix<float, Eigen::Dynamic, rotColDefs::NUM_ROTDEFS_FLOAT> tblBasic;
					// Matrix and vector tables get written out also as arrays
					const size_t matSize = 9 * rotColDefs::NUM_MATRIXDEFS;
					const size_t vecSize = 4 * rotColDefs::NUM_VECTORDEFS;
					Eigen::Matrix<float, Eigen::Dynamic, matSize> tblMatrices;
					Eigen::Matrix<float, Eigen::Dynamic, vecSize> tblVectors;

					const size_t rows = s->rotstats.size();
					tblBasic.resize(rows, rotColDefs::NUM_ROTDEFS_FLOAT);
					tblMatrices.resize(rows, matSize);
					tblVectors.resize(rows, vecSize);

					size_t row = 0;
					for (const auto &rot : s->rotstats)
					{
						//write_hdf5_statsrotatedrawdata(grpRotations, rot.get());
						const basicTable &tbl = rot.get<0>();
						const matrixTable &mat = rot.get<1>();
						const vectorTable &vec = rot.get<2>();

						for (size_t i = 0; i < rotColDefs::NUM_ROTDEFS_FLOAT; ++i)
							tblBasic(row, i) = tbl[i];
						vectorize(mat, tblMatrices, row);
						vectorize(vec, tblVectors, row);

						++row;
					}

					auto dBasic = addDatasetEigen(grpRotations, "Basic", tblBasic, make_plist(rows, rotColDefs::NUM_ROTDEFS_FLOAT));
					auto dMats = addDatasetEigen(grpRotations, "Matrices", tblMatrices, make_plist(rows, matSize));
					auto dVecs = addDatasetEigen(grpRotations, "Vectors", tblVectors, make_plist(rows, vecSize));

					addColNames(dBasic, rotColDefs::NUM_ROTDEFS_FLOAT, rotColDefs::stringifyBasic);
					addColNames(dMats, 9 * rotColDefs::NUM_MATRIXDEFS, rotColDefs::stringifyMatrix, 9, 3);
					addColNames(dVecs, 4 * rotColDefs::NUM_VECTORDEFS, rotColDefs::stringifyVector, 4);
				}

				return statsraw;
			}

		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;

		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<rtmath::phaseFuncs::pfRunSetContainer>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const boost::shared_ptr<const rtmath::phaseFuncs::pfRunSetContainer > s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h = registry::construct_handle
				<registry::IOhandler, hdf5_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
				new hdf5_handle(filename.c_str(), iotype)); });

			// Check for the existence of the appropriate:
			shared_ptr<Group> grpNonDDA = openOrCreateGroup(h->file, "NonDDA");
			// Group "Hashed"/shp->hash
			shared_ptr<Group> grpKey = openOrCreateGroup(grpNonDDA, key.c_str());
			// Group "Hashed"/shp->hash/"Shape". If it exists, overwrite it. There should be no hard links here.
			/// \note The unlink operation does not really free the space..... Should warn the user.
			//if (groupExists(grpKey, "Stats")) return h; //grpHash->unlink("Stats");

			/// \todo Modify to also support external symlinks
			shared_ptr<Group> res = write_hdf5_pfrawdata(grpKey, s);

			return h; // Pass back the handle
		}

	}
}
