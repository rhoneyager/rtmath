#include "Stdafx.h"
#include <memory>
#include "ddscat.h"
#include <iostream>
#include <sstream>
#include "phaseFunc.h"

namespace rtmath {

namespace ddscat {

	ddRun::ddRun()
	{
		_geom = GEOMETRY::NONE;
		_method = SOLUTIONMETHOD::NONE;
		for (size_t i=0;i<3;i++)
		{
			_shapeParams[i] = 1;
			_rotBeta[i] = 0;
			_rotTheta[i] = 0;
			_rotPhi[i] = 0;
			_iStart[i] = 0;
			_memInit[i] = 200;
		}
		for (size_t i=0;i<4;i++)
		{
			_wavelengths[i] = 0;
			_Reff[i] = 0;
		}
		for (size_t i=0;i<6;i++)
			_incidPol[i] = 0;
		_frameTarget = false;

		_prelimOptions.push_back("NOTORQ");
		_prelimOptions.push_back("PBCGS2");
		_prelimOptions.push_back("GPFAFT");
		_prelimOptions.push_back("LATTDR");
		_prelimOptions.push_back("NOTBIN");

	}

	ddRun::~ddRun()
	{

	}

	ddOutput::ddOutput()
	{
		_valid = false;
		_numDipoles = 0;
		_daeff = 0;
		_d = 0;

	}

	void ddOutput::writeEvans(std::string filename)
	{

	}

	void ddOutput::readEvans(std::string filename)
	{

	}

	void ddOutput::readdir(std::string dirpath)
	{
		// Begin by searcing for ddscat.par
	}

	void ddOutput::writedir(std::string dirpath)
	{

	}

	ddOutputSingle::ddOutputSingle(double beta, double theta, double phi)
	{
		_init();
		_beta = beta;
		_theta = _theta;
		_phi = phi;
	}

	void ddOutputSingle::_init()
	{
		_wavelength = 0;
		_sizep = 0;
		_reff = 0;
		_d = 0;
		_numDipoles = 0;
	}

	void ddOutputSingle::setF(const ddCoords &coords, const ddOutputScatt &f)
	{
		_fijs[coords] = f;
	}

	void ddOutputSingle::setS(const ddCoords &coords, const ddOutputMueller &s)
	{
		_Sijs[coords] = s;
	}

	void ddOutputSingle::getF(const ddCoords &coords, ddOutputScatt &f) const
	{
		f = _fijs[coords];
	}

	void ddOutputSingle::getS(const ddCoords &coords, ddOutputMueller &s) const
	{
		s = _Sijs[coords];
	}

	void ddOutputSingle::calcS()
	{
		// Use the standard matrix relationships to extract S from f
		// This really just calls the code found in phaseFunc.cpp's
		// scattMatrix::_genMuellerMatrix(double Snn[4][4], complex<double> Sn[4])
		std::map<ddCoords,ddOutputScatt,ddOutComp>::const_iterator it;
		_Sijs.clear(); // clear the existing Sijs
		for (it = _fijs.begin(); it != _fijs.end(); it++)
		{
			ddOutputMueller mueller;
			ddCoords crds = it->first;
			scattMatrix::_genMuellerMatrix(mueller.Sij, it->second.f);
			_Sijs[crds] = mueller;
		}
	}

	void ddOutputSingle::loadFile(const std::string &fileheader)
	{
		// Open the desired ddscat files and read in the values
		using namespace std;

		// The prefix of the ddscat names is to be passed to the program. The two
		// files opened are the prefix and '.fml' and '.sca' (if needed)
		ostringstream strFml;
		strFml << fileheader << ".fml";
		ifstream infml(strFml.str().c_str());

		// Let's just assume that the input is standard, without missing lines, and is the latest format
		// In this case, Beta is on line 21, Theta 22, Phi 23.
		// Wavelength 8, reff 7, numDipoles 2, sizep 9, n 10
		// The data begin on line 29

		double Beta, Theta, Phi, theta, phi, re, im;
		double wvlen, reff, sizep;

		string linein;

	}

}; // end namespace ddscat
}; // end namespace rtmath

