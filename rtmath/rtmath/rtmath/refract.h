#pragma once
#include "defs.h"
#include <complex>
#include <functional>
#include <string>
#include <vector>

namespace boost { namespace program_options { class options_description; class variables_map; } }
namespace rtmath {
	namespace refract {
		/// m to e converters
		void DLEXPORT_rtmath_core mToE(std::complex<double> m, std::complex<double> &e);
		void DLEXPORT_rtmath_core eToM(std::complex<double> e, std::complex<double> &m);

		/// Dielectric providers - these use f and T to automatically determine the correct
		/// base dielectric function to use.
		void DLEXPORT_rtmath_core mWater(double f, double t, std::complex<double> &m);
		void DLEXPORT_rtmath_core mIce(double f, double t, std::complex<double> &m);


		// The raw dielectric providers

		/// Water complex refractive index for microwave for 0 to 1000 GHz
		/// Liebe, Hufford and Manabe (1991)
		void DLEXPORT_rtmath_core mWaterLiebe(double f, double t, std::complex<double> &m);

		/// Ice complex refractive index
		/// Christian Matzler (2006)
		void DLEXPORT_rtmath_core mIceMatzler(double f, double t, std::complex<double> &m);

		/// Ice complex refractive index for microwave/uv
		void DLEXPORT_rtmath_core mIceWarren(double f, double t, std::complex<double> &m);

		/// Water complex refractive index for ir/vis
		void DLEXPORT_rtmath_core mWaterHanel(double lambda, std::complex<double> &m);

		/// Ice complex refractive index for ir/vis
		void DLEXPORT_rtmath_core mIceHanel(double lambda, std::complex<double> &m);

		/// Sodium chloride refractive index for ir/vis
		void DLEXPORT_rtmath_core mNaClHanel(double lambda, std::complex<double> &m);

		/// Sea salt refractive index for ir/vis
		void DLEXPORT_rtmath_core mSeaSaltHanel(double lambda, std::complex<double> &m);

		/// Dust-like particle refractive index for ir/vis
		void DLEXPORT_rtmath_core mDustHanel(double lambda, std::complex<double> &m);

		/// Sand O-ray refractvie index for ir/vis (birefringent)
		void DLEXPORT_rtmath_core mSandOHanel(double lambda, std::complex<double> &m);

		/// Sane E-ray refractive index for ir/vis (birefringent)
		void DLEXPORT_rtmath_core mSandEHanel(double lambda, std::complex<double> &m);


		/// basic Liu-based diel.tab writer
		/// Deprecated
		void DLEXPORT_rtmath_core DEPRECATED writeDiel(const std::string &filename, 
			const std::complex<double> &m);

		// Refractive index transformations
		// These are used when the ice crystals contain air or water
		// With given volume fractions (f).
		

		/// Bruggeman (1935) / Landauer (1952) / Polder and van Santen (1946) / Effective Medium
		/// See Bohren and Battan (1980)
		void DLEXPORT_rtmath_core bruggeman(std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres);

		/// Bohren and Battan (1980)
		/// Debye formula (specific case of Clausius-Mosotti), Debye(1929, pp. 44-47)
		void DLEXPORT_rtmath_core debyeDry(std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres);



		// Maxwell-Garnett - assuming sphere inclusions with standard ice / water / air dielectric
		//void maxwellGarnettSpheresMulti(std::complex<double> Mice, std::complex<double> Mwater, std::complex<double> Mair, double fIce, double fWater, std::complex<double> &Mres);

		/// Maxwell-Garnett - assuming that ice spheres are inclusions and water is the surrounding medium
		void DLEXPORT_rtmath_core maxwellGarnettSpheres(std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres);

		/// Maxwell-Garnett - assuming ellipsoidal inclusions
		/// Bohren and Battan (1982). Ma for ice, mB for air.
		void DLEXPORT_rtmath_core maxwellGarnettEllipsoids(std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres);

		/// Sihvola (1989) - requires extra parameter, nu.
		/// nu = 0 -> Maxwell-Garnett for spherical inclusions
		/// nu = 2 -> Bruggeman
		/// Petty and Huang (2010) use nu = 0.85.
		void DLEXPORT_rtmath_core sihvola(std::complex<double> Ma, std::complex<double> Mb, double fa, double nu, std::complex<double> &Mres);

		struct sihvolaBinder
		{
			sihvolaBinder(double nu = 0.85) : nu(nu) {}
			double nu;
			void calcSihvola(std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres) const
			{
				sihvola(Ma,Mb,fa,nu,Mres);
			}
			// std::bind compatability
			void operator()(std::complex<double> Ma, std::complex<double> Mb, double fa, std::complex<double> &Mres) const
			{
				sihvola(Ma,Mb,fa,nu,Mres);
			}
		};



		typedef std::function<void(std::complex<double>, std::complex<double>, double, std::complex<double>&)>
			basicDielectricTransform;

		// Multiphase inclusions
		// This function chains dielectrics (such as for 3-phase MG) and reports the overall refractive index.
		// The function form requires binding for more complex cases, such as the sihvola formulation.
		void MultiInclusions(const std::vector< basicDielectricTransform >&,
			const std::vector<double> &fs, const std::vector<std::complex<double> > &ms, std::complex<double> &Mres);

		// And as a template
		/*
		template <typename... ordering>
		void MultiInclusionsT(ordering... vals)
		{
		}
		*/

		// 3-phase MG aliases



		// Function that returns the appropriate refractive index calculator based on a string



		// Converter function bindings for volume fraction / density fraction



		// Temperature-guessing
		double DLEXPORT_rtmath_core guessTemp(double freq, const std::complex<double> &mToEval,
			std::function<void(double freq, double temp, std::complex<double>& mres)> meth = rtmath::refract::mIce);

		/**
		* \brief Adds options to a program
		*
		* \item cmdline provides options only allowed on the command line
		* \item config provides options available on the command line and in a config file
		* \item hidden provides options allowed anywhere, but are not displayed to the user
		**/
		void DLEXPORT_rtmath_core add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/// Processes static options defined in add_options
		/// \todo Add processor for non-static options
		void DLEXPORT_rtmath_core process_static_options(
			boost::program_options::variables_map &vm);

	}
}
