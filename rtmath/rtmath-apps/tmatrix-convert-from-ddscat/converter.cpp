#include <sstream>
#include "converter.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../deps/tmatrix/src/headers/tmatrix.h"

fileconverter::fileconverter()
{
	temp = 0;
}

void fileconverter::setDDPARfile(const std::string &file)
{
	ddparFile = file;
}

void fileconverter::setTemp(double T)
{
	temp = T;
}

void fileconverter::setShapeMethod(const std::string &meth)
{
	shapeMeth = meth;
}

void fileconverter::setDielMethod(const std::string &meth, double nu)
{
	this->nu = nu;
	dielMeth = meth;
}

void fileconverter::setVolFracMethod(const std::string &meth)
{
	volMeth = meth;
}

void fileconverter::setStats(boost::shared_ptr<rtmath::ddscat::shapeFileStats> s)
{
	stats = s;
}

void fileconverter::convert(const std::string &outfile) const
{
	using namespace std;

	// Get volume fraction from the shape statistics. This has no bearing on the 
	// other dimensioning calculations. It is purely for the dielectric calculation.
	double frac;
	{
		if (volMeth == "Minimal circumscribing sphere")
		{
			frac = stats->f_circum_sphere;
		} else if (volMeth == "Convex hull")
		{
			frac = stats->f_convex_hull;
		} else if (volMeth == "Max Ellipsoid")
		{
			frac = stats->f_ellipsoid_max;
		} else if (volMeth == "RMS Ellipsoid")
		{
			frac = stats->f_ellipsoid_rms;
		} else {
			throw rtmath::debug::xBadInput(volMeth.c_str());
		}
	}

	// Calculate shape dimensioning (ellipsoid, sphere, ...)
	{
		// Parse shapeMethod to figure this out
		if (shapeMeth == "Equiv Aeff Sphere")
		{
		} else if (shapeMeth == "Same RMS aspect ratio")
		{
		} else if (shapeMeth == "Same real aspect ratio")
		{
		} else {
			throw rtmath::debug::xBadInput(shapeMeth.c_str());
		}
	}

	// Read ddscat.par file
	// Extract frequencies, reff, rotations, scattering angles and incident vectors
	rtmath::ddscat::ddPar par(ddparFile);
	set<double> freqs; // GHz
	set<double> wavelengths; // um
	{
		double min, max;
		size_t n;
		string spacing;
		par.getWavelengths(min,max,n,spacing);
		// Wavelengths are in microns. Convert interval to frequencies
		ostringstream s;
		s << min << ":" << n << ":" << max << ":" << spacing;
		string ss(s.str());
		
		rtmath::config::splitSet<double>(ss,wavelengths);
		rtmath::units::conv_spec c("um","GHz");
		for (auto it = wavelengths.begin(); it != wavelengths.end(); it++)
		{
			double freq = c.convert(*it);
			if (freqs.count(freq) == 0)
				freqs.insert(freq);
		}
	}

	// reff is needed, since the pure shapefile gives everything in terms of the interdipole spacing
	set<double> aeffs;
	{
		double min, max;
		size_t n;
		string spacing;
		par.getAeff(min,max,n,spacing);
		// Aeffs are in microns. Convert interval to frequencies
		ostringstream s;
		s << min << ":" << n << ":" << max << ":" << spacing;
		string ss(s.str());
		rtmath::config::splitSet<double>(ss,aeffs);
	}

	// rotations and scattering angles combine to get overall rotation
	{
	}

	// Calculate refractive index
	complex<double> mRes;
	{
		complex<double> mIce;
		complex<double> mAir(1.0,0); // TODO: use exact refractive index
		rtmath::refract::mice(*(freqs.begin()), temp, mIce);
		if (dielMeth == "Sihvola")
		{
			rtmath::refract::sihvola(mIce,mAir,frac,nu,mRes);
		} else if (dielMeth == "Debye")
		{
			rtmath::refract::debyeDry(mIce,mAir,frac,mRes);
		} else if (dielMeth == "Maxwell-Garnett")
		{
			rtmath::refract::maxwellGarnettSimple(mIce,mAir,frac,mRes);
		} else {
			throw rtmath::debug::xBadInput(dielMeth.c_str());
		}
	}

	// Write output
	{
		using namespace tmatrix;
		tmatrixInVars in;
		in.AXI; // equiv vol sphere radius
		in.RAT = 1;
		in.LAM = *(wavelengths.begin()); // incident light wavelength
		in.MRR = mRes.real(); // real refractive index
		in.MRI = mRes.imag(); // imag refractive index
		in.NP = -1; // shape is a spheroid
		in.EPS; // ratio of horizontal to rotational axes; >1 for oblate, <1 for prolate

		// And vary ALPHA, BETA, THET0, THET, PHI0, PHI
		// ALPHA, BETA are Euler angles specifying the orientation of the scattering particle 
		// relative to the lab frame
		// Theta is the zenith angle
		// Phi is the azimuth angle
		// _0 is the incident beam, without is the scattered beam

	}
}

