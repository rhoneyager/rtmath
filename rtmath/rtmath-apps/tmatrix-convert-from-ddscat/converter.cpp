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
#include "../../deps/tmatrix/src/headers/tmatrix-serialization.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath-libs/rtmath-mie/mie.h"
#include "../../rtmath-libs/rtmath-mie/mie-serialization.h"

#pragma comment(lib,"tmatrix-cpp")

fileconverter::fileconverter()
{
	temp = 0;
	frequency = 0;
	force180 = false;
	flipS = false;
	dipoleSpacing = 0;
	mieOrder = -1;
}

void fileconverter::setFlipS(bool val)
{
flipS = val;
}

void fileconverter::setForce180(bool val)
{
	force180 = val;
}

void fileconverter::setDDPARfile(const std::string &file)
{
	ddparFile = file;
}

void fileconverter::setDielFile(const std::string &file)
{
	dielFile = file;
}

void fileconverter::setTemp(double T)
{
	temp = T;
}

void fileconverter::setDipoleSpacing(double d)
{
	dipoleSpacing = d;
}

void fileconverter::setFreq(double f)
{
	frequency = f;
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

void fileconverter::setMie(bool a, int val)
{
	if (!a) mieOrder = -1;
	else mieOrder = val;
}

void fileconverter::convert(const std::string &outfile) const
{
	using namespace std;
	// NOTE: this is the isotropic converter. Conversion with angle weightings 
	// requires a precalculation of the different energies
	std::set<double> aeffs, freqs, wavelengths, betas, thetas, phis;
	std::map<double, std::set<double> > angles;
	double frac, asp;

	rtmath::ddscat::ddPar par(ddparFile);
	boost::shared_ptr<rtmath::ddscat::dielTab> dt;
	if (dielFile.size()) dt = boost::shared_ptr<rtmath::ddscat::dielTab>(
		new rtmath::ddscat::dielTab(ddparFile));

	ReadDDPAR(par, freqs, wavelengths, betas, thetas, phis, angles);
	getAsp(asp);

	getAeff(frac, aeffs, par);

	// Calculate and write output
	// need to iterate over aeff, freq
	std::vector<rtmath::tmatrix::tmData> tdSet;
	cerr << "There are " << freqs.size() << " frequencies.\n";
	cerr << "There are " << aeffs.size() << " sizes.\n";
	for (auto freqit = freqs.begin(); freqit != freqs.end(); ++freqit)
		for (auto aeffit = aeffs.begin(); aeffit != aeffs.end(); ++aeffit)
		{
			const double aeff = *aeffit, freq = *freqit;
			const double pi = boost::math::constants::pi<double>();

			cerr << "aeff: " << aeff << ", freq: " << freq << ", d: " << dipoleSpacing << endl;
//			getAeff(frac, aeffs, par);
			complex<double> mRes;

			getM(mRes, frac, freq, dt);
			using namespace tmatrix;
			::rtmath::tmatrix::tmData td;
			// dipole spacing may be recomputed from aeff. d = aeff / aeff_dipoles_const
			double d = aeff / stats->aeff_dipoles_const;
//			if (dipoleSpacing) d = dipoleSpacing;

			rtmath::units::conv_spec c("GHz","um");
			double wvlen = c.convert(freq);

			double sizep = 2. * pi * aeff / wvlen;

			td.dipoleSpacing = d;
			td.sizep = sizep;
			td.T = temp;
			td.freq = freq;
			td.nu = nu;
			td.reff = mRes;
			td.volMeth = volMeth;
			td.dielMeth = dielMeth;
			td.shapeMeth = shapeMeth;
			if (mieOrder >= 0)
			{
				cerr << "Performing mie processing instead of tmatrix.\n";
				ostringstream om;
				om << "mie-" << mieOrder;
				td.shapeMeth = om.str();
			} else {
				cerr << "Performing tmatrix processing.\n";
			}
			td.angleMeth = "Isotropic";
			td.ddparpath = ddparFile;
			td.dielpath = dielFile;

			td.stats = stats;

			// Vary ALPHA, BETA, THET0, THET, PHI0, PHI
			// ALPHA, BETA are Euler angles specifying the orientation of the scattering particle 
			// relative to the lab frame
			// Theta is the zenith angle
			// Phi is the azimuth angle
			// _0 is the incident beam, without is the scattered beam
			const double thet0 = 0;
			const double phi0 = 0;

			// TODO: rewrite this to match DDSCAT conventions - individual rotations are treated separately to get 
			// cross-sections and are then averaged together for the isotropic case.
			// Scattering and extinction cross-sections shouldn't be much more involved, per T-matrix calculation.

			// Track number of calculations needed and perform iso averaging for Qsca, Qext, Qbk
			size_t numOri = phis.size() * thetas.size();
			cerr << "no: " << numOri << endl;
			size_t numAngles = 0;
			double Qsca = 0, Qext = 0, Qbk = 0, Walb = 0;

			boost::shared_ptr<const ::tmatrix::tmatrixParams> tp = 
				::tmatrix::tmatrixParams::create(aeff, // equiv vol sphere radius
				1.0, // Indicates that AXI is the equiv volume radius, not equiv sa radius
				wvlen, // incident light wavelength
				mRes.real(), // real refractive index
				abs(mRes.imag()), // imag refractive index
				asp, // ratio of horizontal to rotational axes; >1 for oblate, <1 for prolate
				-1, // shape is a spheroid
				0.001, // solution convergence precision
				4 // number of integrals
				);

			boost::shared_ptr<const rtmath::mie::mieParams> mp = 
				rtmath::mie::mieParams::create(aeff,
				wvlen, mRes.real(), mRes.imag());

			for (auto p = phis.begin(); p != phis.end(); ++p)
			{
				double alpha = *p;
				for (auto t = thetas.begin(); t != thetas.end(); ++t)
				{
					double beta = *t;
					
					boost::shared_ptr<const ::tmatrix::OriTmatrix> oritmm;
					boost::shared_ptr<const rtmath::mie::mieCalc> miec;
					if (mieOrder < 0)
					{
						oritmm = ::tmatrix::OriTmatrix::calc(tp, alpha, beta);
						Qbk += ::tmatrix::getDifferentialBackscatterCrossSectionUnpol(oritmm, flipS) / (pi * aeff * aeff);
						Qext += oritmm->qext;
						Qsca += oritmm->qsca;
						Walb += oritmm->walb;
					} else {
						miec = rtmath::mie::mieCalc::calc(mp);
						Qbk += rtmath::mie::getDifferentialBackscatterCrossSectionUnpol(miec) / (pi * aeff * aeff);
						Qext += miec->qext;
						Qsca += miec->qsca;
						Walb += miec->walb;
					}
					for (auto it = angles.begin(); it != angles.end(); ++it)
					{
						double phi = it->first;
						for (auto ot = it->second.begin(); ot != it->second.end(); ++ot)
						{
							numAngles++;
							double thet = *ot;

							if (mieOrder < 0)
							{
								boost::shared_ptr<const ::tmatrix::OriAngleRes> orires
									= ::tmatrix::OriAngleRes::calc(oritmm, thet, thet0, phi, phi0);

								td.data.push_back(std::move(orires));
							} else {
								boost::shared_ptr<const rtmath::mie::mieAngleRes> orires
									= rtmath::mie::mieAngleRes::calc(miec,thet);
								td.miedata.push_back(std::move(orires));
							}
						} // angle (theta) iteration
					} // angle (phi) iteration
				} // betas
			} // alphas

			// Store global isotropic orientation data
			if (!numAngles) numAngles = 1;
			if (!numOri) numOri = 1;
			Qsca /= (double) numOri;
			Qext /= (double) numOri;
			Qbk /= (double) numOri;
			Walb /= (double) numOri;
			std::map<std::string, double> s;
			s["Qsca"] = Qsca;
			s["Qext"] = Qext;
			s["Qbk"] = Qbk;
			s["Walb"] = Walb;

			td.tstats->stats = s;

			tdSet.push_back(move(td));
			std::cerr << "Added\n";
		}
	// TODO: use prefixes to split the file structures.
	rtmath::serialization::write<std::vector<rtmath::tmatrix::tmData> >
		(tdSet, outfile);
}

