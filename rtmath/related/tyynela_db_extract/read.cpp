/// \brief Provides hdf5 file IO
#define _SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
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

void readFile(const std::string &cross, const std::string &phase,
	const std::string &phys, std::vector<data_entry> &out) {
	using std::shared_ptr;
	using std::string;
	using std::endl;
	using std::cerr;
	using namespace H5;
	using namespace Ryan_Debug;
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::hdf5;

	// Guess frequency from filename
	std::string band, atype;
	double freq = 0, wave = 0, dspacing = 20;
	{
		const string idtag = "Tyynela_240614_cross_";
		size_t pos = cross.find(idtag);
		if (pos == string::npos) throw;
		size_t start = pos+idtag.size();
		size_t end = cross.find("_",start+1);
		band = cross.substr(pos+idtag.size(),end-start);
		start = end+1;
		end = cross.find_last_of(".");
		atype = cross.substr(start, end - start);

		cerr << "band " << band << "  atype " << atype << endl;

		if (band == "G") freq = 220;
		else if (band == "W") freq = 94;
		else if (band == "Ka") freq = 35.6;
		else if (band == "Ku") freq = 13.6;
		else if (band == "X") freq = 9.8;
		else if (band == "C") freq = 5.6;
		else if (band == "S") freq = 2.7;
		else { cerr << "Unknown band" << endl; throw; }

		wave = 2.9979e5 / freq;
		if (atype == "ferndend_aggregate") dspacing = 40;
	}

	const IOhandler::IOtype iotype = IOhandler::IOtype::READONLY;
	//Exception::dontPrint();
	cerr << "Opening " << cross << endl;
	std::shared_ptr<hdf5_handle> hC = registry::construct_handle
		<registry::IOhandler, hdf5_handle>(
		nullptr, "hdf5-read", [&](){return std::shared_ptr<hdf5_handle>(
		new hdf5_handle(cross.c_str(), iotype)); });
	cerr << "Opening " << phase << endl;
	std::shared_ptr<hdf5_handle> hP = registry::construct_handle
		<registry::IOhandler, hdf5_handle>(
		nullptr, "hdf5-read", [&](){return std::shared_ptr<hdf5_handle>(
		new hdf5_handle(phase.c_str(), iotype)); });
	cerr << "Opening " << phys << endl;
	std::shared_ptr<hdf5_handle> hY = registry::construct_handle
		<registry::IOhandler, hdf5_handle>(
		nullptr, "hdf5-read", [&](){return std::shared_ptr<hdf5_handle>(
		new hdf5_handle(phys.c_str(), iotype)); });

	cerr << "Reading matrices" << endl;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> arrt;
	arrt chh, cvv, csca, cabs, cext, asym, ncryst, ndip, dmax, ar, mass;

	readDatasetEigen<arrt, H5::H5File>(hC->file, "chh", chh);
	readDatasetEigen<arrt, H5::H5File>(hC->file, "cvv", cvv);
	readDatasetEigen<arrt, H5::H5File>(hC->file, "csca", csca);
	readDatasetEigen<arrt, H5::H5File>(hC->file, "cabs", cabs);
	readDatasetEigen<arrt, H5::H5File>(hC->file, "cext", cext);
	readDatasetEigen<arrt, H5::H5File>(hC->file, "asym", asym);
	readDatasetEigen<arrt, H5::H5File>(hY->file, "ncryst", ncryst);
	readDatasetEigen<arrt, H5::H5File>(hY->file, "ndip", ndip);
	readDatasetEigen<arrt, H5::H5File>(hY->file, "dmax", dmax);
	readDatasetEigen<arrt, H5::H5File>(hY->file, "ar", ar);
	readDatasetEigen<arrt, H5::H5File>(hY->file, "mass", mass);

	// Each row/column combination is a separate entry in the tables
	// Need to determine effective radius from mass first to normalize the cross-sections.
	// Initial mass in in kg.
	const double rho0 = 917.0; // density of solid ice in kg/m^3
	const double pi = boost::math::constants::pi<double>();
	arrt vol = mass / rho0; // volume in m^3
	// aeff should be in microns. These two ways are equivalent.
	arrt aeff = ((3.*(vol.array())/(4.*pi)).pow(1./3.)).matrix() * 1.e6;
	arrt aeffb = ( (3./(4.*pi)) * pow(dspacing,3.) * ndip.array() ).pow(1./3.).matrix();
	arrt cbk = chh;
	arrt qsca = ((csca.array()/(pi*(aeff.array()*aeff.array())))).matrix();
	arrt qabs = ((cabs.array()/(pi*(aeff.array()*aeff.array())))).matrix();
	arrt qext = ((cext.array()/(pi*(aeff.array()*aeff.array())))).matrix();
	arrt qbk = ((cbk.array()/(pi*(aeff.array()*aeff.array())))).matrix();

	for (int row = 0; row < chh.rows(); ++row)
		for (int col = 0; col < chh.cols(); ++col)
		{
			std::ostringstream tag;
			tag << "tyn_" << atype << "_" << row << "_" << col;
			data_entry d;
			d.source = atype;
			d.id = tag.str();
			d.freq = freq; d.wave = wave; d.dspacing = dspacing;
			d.ar = ar(row,col);
			d.aeff = aeffb(row,col); // in um
			d.md = dmax(row,col) * 1000; // in mm
			d.g = asym(row,col);
			d.qabs = qabs(row,col); d.qbk = qbk(row,col);
			d.qext = qext(row,col); d.qsca = qsca(row,col);
			out.push_back(std::move(d));
		}
}

