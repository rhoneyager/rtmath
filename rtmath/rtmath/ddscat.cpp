#include "Stdafx.h"
#include <memory>
#include "ddscat.h"
#include <iostream>
#include <sstream>

namespace rtmath {

	ddRun::ddRun()
	{
		_geom = NONE;
		_method = NONE;
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

	ddOutput::writeEvans(std::string filename)
	{

	}

	ddOutput::readEvans(std::string filename)
	{

	}

	ddOutput::readdir(std::string dirpath)
	{
		// Begin by searcing for ddscat.par
	}

	ddOutput::writedir(std::string dirpath)
	{

	}

}; // end namespace rtmath

