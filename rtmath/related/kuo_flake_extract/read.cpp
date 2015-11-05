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
#include <boost/math/constants/constants.hpp>

#include <Ryan_Debug/error.h>
#include <Ryan_Debug/registry.h>

#include <hdf5.h>
#include <H5Cpp.h>
#include "../rtmath_hdf5_cpp/export-hdf5.h"

#include "read.h"

struct hdf5_handle : public Ryan_Debug::registry::IOhandler
{
	hdf5_handle(const char* filename, IOtype t);
	virtual ~hdf5_handle() {}
	void open(const char* filename, IOtype t);
	std::shared_ptr<H5::H5File> file;
};
hdf5_handle::hdf5_handle(const char* filename, IOtype t)
	: IOhandler("hdf5-reader")
{
	open(filename, t);
}

void hdf5_handle::open(const char* filename, IOtype t)
{
	using namespace H5;
	switch (t)
	{
	case IOtype::READWRITE:
		file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_RDWR ));
		break;
	case IOtype::EXCLUSIVE:
		file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_EXCL ));
		break;
	case IOtype::DEBUG:
		file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_DEBUG ));
		break;
	case IOtype::CREATE:
		file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_CREAT ));
		break;
	case IOtype::READONLY:
		file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_RDONLY ));
		break;
	case IOtype::TRUNCATE:
		file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_TRUNC ));
		break;
	}
}
/*
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
*/
void readFile(const std::string &inFile, std::vector<data_entry> &out) {
	using std::shared_ptr;
	using std::string;
	using std::endl;
	using std::cerr;
	using namespace H5;
	using namespace Ryan_Debug;
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::hdf5;

	cerr << "Reading file " << inFile << endl;
	const IOhandler::IOtype iotype = IOhandler::IOtype::READONLY;
	//Exception::dontPrint();
	std::shared_ptr<hdf5_handle> h = registry::construct_handle
		<registry::IOhandler, hdf5_handle>(
		nullptr, "hdf5-read", [&](){return std::shared_ptr<hdf5_handle>(
		new hdf5_handle(inFile.c_str(), iotype)); });

	shared_ptr<Group> grpHashes = openGroup(h->file, "shape");
	if (grpHashes)
	{
		hsize_t sz = grpHashes->getNumObjs();
		//s.reserve(s.size() + sz);
		for (hsize_t i = 0; i < sz; ++i)
		{
			std::string hname = grpHashes->getObjnameByIdx(i);
			H5G_obj_t t = grpHashes->getObjTypeByIdx(i);
			if (t != H5G_obj_t::H5G_GROUP) continue;

			shared_ptr<Group> grpHash = openGroup(grpHashes, hname.c_str());
			if (!grpHash) { continue; } // Should never happen
			shared_ptr<Group> grpRuns = openGroup(grpHash, "size");
			if (!grpRuns) { continue; }

			hsize_t rz = grpRuns->getNumObjs();
			for (hsize_t i = 0; i < rz; ++i)
			{
				std::string runname = grpRuns->getObjnameByIdx(i);
				H5G_obj_t t = grpRuns->getObjTypeByIdx(i);
				if (t != H5G_obj_t::H5G_GROUP) continue;
				shared_ptr<H5::Group> grpRun = openGroup(grpRuns, runname.c_str());
				if (!grpRun) { continue;}

				// subfolders are frequency and pblock
				shared_ptr<H5::Group> grpPblock = openGroup(grpRun, "pblock");
				if (!grpPblock) { continue;}

				double md = 0, aeff = 0, dspacing = 0, ar = 0, mass = 0;
				readAttr<double, Group>(grpPblock, "INTER_DIPOLE_DISTANCE", dspacing);

				typedef Eigen::Matrix<float, Eigen::Dynamic, 1> arrt;
				arrt mat; mat.resize(1,1);
				readDatasetEigen<arrt, Group>(grpPblock, "EVOL_RADIUS", mat);
				aeff = mat(0,0);

				readDatasetEigen<arrt, Group>(grpPblock, "MAX_DIM", mat);
				md = mat(0,0);

				mat.resize(3,1);
				readDatasetEigen<arrt, Group>(grpPblock, "CIRC_ELLIPSOID_SEMI_AXIS_LENGTH", mat);
				ar = 2 * mat(0,0) / (mat(1,0) + mat(2,0) );

				// iterate over frequency
				shared_ptr<H5::Group> grpFreqs = openGroup(grpRun, "frequency");
				if (!grpFreqs) continue;
				hsize_t fz = grpFreqs->getNumObjs();
				for (hsize_t f = 0; f < fz; ++f)
				{
					std::string freqname = grpFreqs->getObjnameByIdx(f);
					H5G_obj_t t = grpFreqs->getObjTypeByIdx(f);
					if (t != H5G_obj_t::H5G_GROUP) continue;
					shared_ptr<H5::Group> grpf = openGroup(grpFreqs, freqname.c_str());
					if (!grpf) continue;

					shared_ptr<H5::Group> grpori = openGroup(grpf, "orientation");
					if (!grpori) continue;
					shared_ptr<H5::Group> grpavg = openGroup(grpori, "average");
					if (!grpavg) continue;
					shared_ptr<H5::Group> grpcomp = openGroup(grpavg, "compact");
					if (!grpcomp) continue;
					shared_ptr<H5::Group> grpadv = openGroup(grpavg, "advanced");
					if (!grpadv) continue;

					// advanced group has orientations as attributes
					// compact group has the cross-sections
					double qabs = 0, qbk = 0, qext = 0, qsca = 0, g = 0, aeffb = 0,
						wave = 0, freq = 0;
					int nb = 0, nt = 0, np = 0;
					readAttr<int, Group>(grpadv, "NBETA", nb);
					readAttr<int, Group>(grpadv, "NTHETA", nt);
					readAttr<int, Group>(grpadv, "NPHI", np);
					readAttr<double, Group>(grpadv, "AEFF", aeffb);
					readAttr<double, Group>(grpadv, "WAVE", wave);
					// Frequency conversion
					freq = (2.9979e5) / wave;
					// Get mass
					const double pi = boost::math::constants::pi<double>();
					double volume = 4. * pi * pow(aeffb,3) / 3.; // in um3
					const double den = 0.917 * 1e-12; // g/um^3
					mass = volume * den; // in g

					readDatasetEigen<arrt, Group>(grpcomp, "Q_abs", mat);
					qabs = mat(0,0);
					readDatasetEigen<arrt, Group>(grpcomp, "Q_bk", mat);
					qbk = mat(0,0);
					readDatasetEigen<arrt, Group>(grpcomp, "Q_ext", mat);
					qext = mat(0,0);
					readDatasetEigen<arrt, Group>(grpcomp, "Q_sca", mat);
					qsca = mat(0,0);
					readDatasetEigen<arrt, Group>(grpcomp, "g", mat);
					g = mat(0,0);

					data_entry d;
					d.version = 0;
					d.id = hname;
					d.nb = nb; d.nt = nt; d.np = np;
					d.aeff = aeffb; d.freq = freq; d.md = md; d.wave = wave; d.dspacing = dspacing;
					d.qabs = qabs; d.qbk = qbk; d.qext = qext; d.qsca = qsca; d.g = g;
					d.mass = mass;
					d.ar = ar;
					out.push_back(std::move(d));
				}

			}
		}
	}
}

