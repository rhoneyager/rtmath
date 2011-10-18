#include "Stdafx.h"
#include <memory>
#include "ddscat.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include "phaseFunc.h"
#include "macros.h"

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

		double Beta, Theta, Phi, re, im;
		double wvlen, reff, sizep;
		size_t numDipoles;
		std::complex<double> n;

		string linein;
		size_t line=0;
		char newval[20];

		while (line!=2)
		{
			std::getline(infml,linein);
			line++;
		}

		// Line 2 is in the string buffer. Now, extract numDipoles
		strncpy(newval, linein.data()+20, 5);
		numDipoles = M_ATOI(newval);
		this->numDipoles(numDipoles);

		// Loop to interesting line (line 7)
		while (line!=7)
		{
			std::getline(infml,linein);
			line++;
		}
		// Line 7 is in the string buffer. Now, extract reff
		// reff is in chars 10-17, inclusively
		strncpy(newval, linein.data()+10, 7);
		reff = M_ATOF(newval);
		this->reff(reff);

		// Line 8 - wavelength
		std::getline(infml,linein);
		strncpy(newval, linein.data()+10, 7);
		wvlen = M_ATOF(newval);
		this->wavelength(wvlen);

		// Line 9 - size parameter
		std::getline(infml,linein);
		strncpy(newval, linein.data()+10, 7);
		sizep = M_ATOF(newval);
		this->sizep(sizep);

		// Line 10 - n
		std::getline(infml,linein);
		strncpy(newval, linein.data()+4, 7);
		re = M_ATOF(newval);
		strncpy(newval, linein.data()+13, 7);
		im = M_ATOF(newval);
		n.real(re);
		n.imag(im);
		this->n.push_back(n);

		line = 10;
		// seek to line 21 for beta
		while (line!=21)
		{
			std::getline(infml,linein);
			line++;
		}

		// Set Beta
		strncpy(newval, linein.data()+10, 7);
		Beta = M_ATOF(newval);
		this->beta(Beta);

		// Set Theta
		std::getline(infml,linein);
		strncpy(newval, linein.data()+10, 7);
		Theta = M_ATOF(newval);
		this->theta(Theta);

		// Set Phi
		std::getline(infml,linein);
		strncpy(newval, linein.data()+10, 7);
		Phi = M_ATOF(newval);
		this->phi(Phi);

		line = 23;
		// Seek to the beginning of the data section
		while (line!=28)
		{
			std::getline(infml,linein);
			line++;
		}

		// Loop until eof and read in the scattering amplitude matrix information
		while (infml.good())
		{
			std::getline(infml,linein);
			double theta, phi; // also using re and im from before
			istringstream parser(linein, istringstream::in);
			parser >> theta;
			parser >> phi;
			ddOutputScatt newdata;
			for (size_t i=0;i<4;i++)
			{
				parser >> re;
				parser >> im;
				newdata.f[i].real(re);
				newdata.f[i].imag(im);
			}
			ddCoords crds(theta,phi);
			if (_fijs.count(crds) == 0)
				_fijs[crds] = newdata;
		}
	}

}; // end namespace ddscat
}; // end namespace rtmath

