#include <algorithm>
#include <sstream>
#include <boost/serialization/vector.hpp>
#include "converter.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/dielTabFile.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/command.h"

#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/Serialization/tmData_serialization.h"

#include "../../deps/tmatrix/src/headers/tmatrix.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"

void fileconverter::getAeff(double &frac, 
	std::set<double> &aeffs, const rtmath::ddscat::ddPar& par) const
{
	// Get volume fraction from the shape statistics. This has no bearing on the 
	// other dimensioning calculations. It is purely for the dielectric calculation.
	// aeff is a united quantity. Needs to be rescaled from dipole coords into microns!
	// I need the interdipole spacing, as read from the ddscat.par file.
	// Note: ddscat.par has the aeff for only the dipoles, not the whole shape!
	// aeffs has the actual effective radius in length units, not dipoles
	
	if (volMeth == "Minimal circumscribing sphere")
	{
		frac = stats->f_circum_sphere;
		//aeff = stats->a_circum_sphere;
	} else if (volMeth == "Convex hull")
	{
		frac = stats->f_convex_hull;
		//aeff = stats->aeff_V_convex_hull;
	} else if (volMeth == "Max Ellipsoid")
	{
		frac = stats->f_ellipsoid_max;
		//aeff = stats->aeff_ellipsoid_max;
	} else if (volMeth == "RMS Ellipsoid")
	{
		frac = stats->f_ellipsoid_rms;
		//aeff = stats->aeff_ellipsoid_rms;
	} else {
		throw rtmath::debug::xBadInput(volMeth.c_str());
	}

	using namespace std;
	double min, max;
	size_t n;
	string spacing;
	par.getAeff(min,max,n,spacing);
	// Aeffs are in microns. Convert interval to frequencies
	ostringstream s;
	std::transform(spacing.begin(),spacing.end(),spacing.begin(), ::tolower);
	s << min << ":" << n << ":" << max << ":" << spacing;
	string ss(s.str());
	std::set<double> dipoleaeffs;
	rtmath::config::splitSet<double>(ss,dipoleaeffs);

	// double aeff_dipoles_const = stats->aeff_dipoles_const;
	// Look at the specified dipoleSpacing value. If provided, take this as the default. 
	// If not, then we can just use the effective radii provided in the ddscat.par file.
	if (dipoleSpacing)
	{
		// Taking the dipole spacing as a constant, and overriding the ddscat.par
		// values for the effective radius. If this option is taken, then only 
		// one effective raduis gets calculated.
		double d = dipoleSpacing;
		//for (auto it = dipoleaeffs.begin(); it != dipoleaeffs.end(); ++it)
		//	aeffs.insert(*it * d);
		double aeff_dipoles = stats->aeff_dipoles_const;
		aeffs.insert(d * aeff_dipoles);
	} else {
		// Using the ddscat.par effective radii.
		aeffs = dipoleaeffs;
	}
}

void fileconverter::ReadDDPAR(const rtmath::ddscat::ddPar& par,
	std::set<double> &freqs, std::set<double> &wavelengths,
	std::set<double> &betas, std::set<double> &thetas, 
	std::set<double> &phis, std::map<double, std::set<double> > &angles) const
{
	using namespace std;
	// Read ddscat.par file
	// Extract frequencies, reff, rotations, scattering angles and incident vectors
	if (!frequency)
	{
		double min, max;
		size_t n;
		string spacing;
		par.getWavelengths(min,max,n,spacing);
		// Wavelengths are in microns. Convert interval to frequencies
		ostringstream s;
		std::transform(spacing.begin(), spacing.end(), spacing.begin(), ::tolower);
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
	} else {
		freqs.insert(frequency);
		rtmath::units::conv_spec c("GHz","um");
		double wvlen = c.convert(frequency);
		wavelengths.insert(wvlen);
	}

	// rotations and scattering angles combine to get overall choice of angles
	//set<double> betas, thetas, phis;
	//map<double, set<double> > angles; // (phi, set of thetas)
	{
		// valid only if cmdfrm is in the lab frame 'LFRAME'
		// throw if tframe (another approach is needed)
		string sframe;
		par.getCMDFRM(sframe);
		if (sframe != "LFRAME") 
			throw rtmath::debug::xBadInput("Only LFRAME supported");

		// The ellipsoid starts oriented with the z axis as the axis of rotation.
		// alpha and beta are then varied to rotate it about this axis. 
		rtmath::ddscat::rotations rots;
		par.getRots(rots);
		//set<double> betas, thetas, phis; // above

		// the ROTATION mapping is ddstat Theta = tmatrix beta
		// tmatrix alpha = ddscat Phi (but not really. tmatrix ordering is different, 
		// and given rot symmetry, it doesn't matter)
		rots.betas(betas);
		rots.thetas(thetas);
		rots.phis(phis);

		stats->calcStatsRot(0,0,0);


		// The scattering plane section.
		size_t n = par.numPlanes();
		for (size_t i=1; i<=n; i++)
		{
			double phi, thetan_min, thetan_max, dtheta;
			par.getPlane(i, phi, thetan_min, thetan_max, dtheta);
			ostringstream sT;
			sT << thetan_min << ":" << dtheta << ":" << thetan_max;
			set<double> scatThetas;
			rtmath::config::splitSet<double>(sT.str(), scatThetas);
			if (angles.count(phi) == 0)
			{
				set<double> ns;
				angles[phi] = ns;
			}
			if (!force180)
			{
				for (auto it = scatThetas.begin(); it != scatThetas.end(); ++it)
					angles[phi].insert(*it);
			}
			else angles[phi].insert(180.0);
		}
	}
}

void fileconverter::getM(std::complex<double> &mRes, double frac, 
	double freq, const boost::shared_ptr<rtmath::ddscat::dielTab> dt) const
{
	using namespace std;
	complex<double> mMed;
	complex<double> mAir(1.0,0); // TODO: use exact refractive index
	if (dielFile.size() == 0)
	{
		rtmath::refract::mice(freq, temp, mMed);
	} else {
		mMed = dt->interpolate(freq);
	}
	// For convention, imaginary part of refractive index is taken to be positive
	mMed = complex<double>(mMed.real(),abs(mMed.imag()));
	// The dielectric signs of mRes need to be positive.
	if (dielMeth == "Sihvola")
	{
		rtmath::refract::sihvola(mMed,mAir,frac,nu,mRes);
	} else if (dielMeth == "Debye")
	{
		rtmath::refract::debyeDry(mMed,mAir,frac,mRes);
	} else if (dielMeth == "Maxwell-Garnett")
	{
		rtmath::refract::maxwellGarnettSimple(mMed,mAir,frac,mRes);
	} else {
		throw rtmath::debug::xBadInput(dielMeth.c_str());
	}
}

void fileconverter::getAsp(double &asp) const
{
	// Initial rotation is about x, which would be the rotational axes
	// So, the horizontal axes are y and z
	// Need to take the mean of two of the axes
	auto r = stats->rotations.begin();
	// Parse shapeMethod to figure this out
	// TODO: need to record stats of the max aspect ratio / maximally distorted 
	// line, which should occur along the two furthest points
	if (shapeMeth == "Equiv Aeff Sphere")
	{
		asp = 1.0;
	} else if (shapeMeth == "Same RMS aspect ratio")
	{
		double x = (*r)->as_rms.get(2,0,0);
		double y = (*r)->as_rms.get(2,1,0);
		double z = (*r)->as_rms.get(2,2,0);

		asp = (0.5 * (y + z)) / x;
		//stats->
	} else if (shapeMeth == "Same real aspect ratio")
	{
		double x = (*r)->as_abs.get(2,0,0);
		double y = (*r)->as_abs.get(2,1,0);
		double z = (*r)->as_abs.get(2,2,0);

		asp = (0.5 * (y + z)) / x;
	} else {
		throw rtmath::debug::xBadInput(shapeMeth.c_str());
	}
}
