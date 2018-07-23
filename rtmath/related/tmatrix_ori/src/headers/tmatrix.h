#pragma once
/* tmatrix - a wrapper class for invoking and, eventually, processing
 * the output of Mischenko's tmatrix functions for extended precision.
 *
 * The C++ and C-style wrapper functions allow for setting initial
 * problem conditions at run time.
 *
 * Note: the code uses COMMON variables. Threads cannot be used to
 * parallelize this code. Instead, separate program invocations
 * are required.
 */

#include "defs.h"
// boost provides the size-explicit data types for C++ compilers that
// lack full C99 support!
#include <boost/cstdint.hpp>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/shared_array.hpp>
#include <set>
#include <map>
#include <array>
#include <vector>

// Need these so the template friends can work
namespace tmatrix
{
	struct tmatrixBase;
	class tmatrixParams;
	class OriTmatrix;
	class OriAngleRes;
	namespace queue
	{
		class TM;
		class TMangles;
		class TMrequest;
	}
}

#pragma warning(push)
#pragma warning(disable: 4251) // needs DLL-interface warning is incorrect

/// The global tmatrix namespace.
namespace tmatrix
{
	/// Base structure for holding T-matrix inputs
	struct DLEXPORT_TMATRIX tmatrixBase
	{
		tmatrixBase();
		double AXI, RAT, LAM,
			MRR, MRI, EPS,
			DDELT, ALPHA, BETA;
		bool isIso;
		boost::int32_t NP, NDGS;
		bool operator<(const tmatrixBase&) const;
	};

	/// Class that sets up the Fortran code for a run with T-matrix inputs.
	class DLEXPORT_TMATRIX tmatrixParams
	{
	public:
		virtual ~tmatrixParams() {}
		/// Create tmatrixParams using structure as a base
		static boost::shared_ptr<const tmatrixParams> create(const tmatrixBase&);
		/** \brief Create tmatrixParams by specifying each parameter
		*
		* \param axi is the equivalent sphere radius.
		* \param rat indicates particle size in terms of equal-volume or equal-sa sphere radius?
		* \param lam is the wavelength of incident light.
		* \param mrr is the real refractive index.
		* \param mri is the imaginary refractive index (positive).
		* \param eps is the ratio of horizontal to rotational axes. >1 for oblate.
		* \param np indicates the particle shape. -1 for spheroids.
		* \param ddelt sets bounds on computational accuracy.
		* \param ndgs indicates the number of integral division points.
		**/
		static boost::shared_ptr<const tmatrixParams> create(
			double axi, /// equiv sphere radius
			double rat, /// is particle size in terms of equal-volume or equal-sa sphere radius?
			double lam, /// wavelength of incident light
			double mrr, /// real refractive index
			double mri, /// imaginary refractive index (positive)
			double eps, /// ratio of horizontal to rotational axes. >1 for oblate
			boost::int32_t np, /// = -1 for spheroids
			double ddelt, /// computational accuracy
			boost::int32_t ndgs, /// number or integral division points
			bool isIso = false /// is isotropic orientation desired?
			);
		double axi, rat, lam,
			eps, ddelt;
		std::complex<double> m; /// Refractive index
		boost::int32_t np, ndgs;
		bool isIso;
		// TODO: add mass calculating function
		tmatrixParams();
		bool operator==(const tmatrixParams&) const;
		bool operator<(const tmatrixParams&) const;
	private:
		bool needApply(double alpha, double beta) const;
		void apply(double alpha, double beta) const;
		bool needApplyIso() const;
		void applyIso() const;
		friend class OriTmatrix;
		friend class OriAngleRes;
	};

	/// Opaque object used to store angle-dependent information for 
	/// an isotropic t-matrix run.
	struct isoRes;

	/**
	 * \brief Performs T-matrix calculations for a given orientation.
	 *
	 * \todo Rotation-dependent cross-sections (need to derive equations in tm form)
	 **/
	class DLEXPORT_TMATRIX OriTmatrix
	{
	public:
		OriTmatrix();
		virtual ~OriTmatrix() {}
		// Need to apply and execute tmat code
		static boost::shared_ptr<const OriTmatrix> calc(
			boost::shared_ptr<const tmatrixParams> in,
			double alpha, double beta);
		static boost::shared_ptr<const OriTmatrix> calcIso(
			boost::shared_ptr<const tmatrixParams> in);
		static boost::shared_ptr<const OriTmatrix> calc(const tmatrixBase&);
		boost::shared_ptr<const tmatrixParams> base;
		// Quantities imported from Mischenko's code
		double qsca; /// Rotation-independent total scattering cross-section
		double qext; /// Rotation-independent total extinction cross-section
		double walb; /// Albedo
		double g; // asymmetry parameter <cos theta>
		double time; /// Computation time
		int nmax;
		// The Q values need rescaling for soft particles. See rtmath plugin.

		// Not imported from Mishchenko
		double alpha, beta; /// Rotation angles
		bool isIso; // Is an isotropic random orientation assumed?
		
		// Using a shared_ptr to avoid stack crushing.
		// Not using complex<double> for ease of copying.
		// Using shared_ptr instead of shared_array for serialization.
		//boost::shared_ptr< const std::vector<float> >
		//	RT11, RT12, RT21, RT22,
		//	IT11, IT12, IT21, IT22;
		// Quantities derived in my code
		//double qbk; // Requires calculation at 180 degrees
		double projArea; /// Projective area (for orientation calculations)
		//bool operator<(const OriTmatrix&) const;
		boost::shared_ptr<isoRes> iso_angle_res;
	private:
		void import(); /// Import from Mischenko Fortran code
		void importIso();
		void calcParams(); /// Calculate my parameters
		void calcParamsIso();
	};

	/**
	 * \brief Class that performs phase function calculation using a given orientation.
	 *
	 * \todo Change accessor functions to use Eigen for speed.
	 **/
	class DLEXPORT_TMATRIX OriAngleRes
	{
	public:
		OriAngleRes();
		virtual ~OriAngleRes() {}
		boost::shared_ptr<const OriTmatrix> tm;
		double theta, theta0, phi, phi0; /// Rotation angles

		std::complex<double> getS(size_t i, size_t j) const;
		double getP(size_t i, size_t j) const;
		double getFlippedP(size_t i, size_t j) const;

		bool operator<(const OriAngleRes &rhs) const;
	private:
		boost::shared_ptr< const std::vector<std::complex<double> > > S;
		boost::shared_ptr< const std::vector<double> > P;
		static boost::shared_ptr< const std::vector<double> > getP(
			const boost::shared_ptr< const std::vector<std::complex<double> > > &S);
	public:
		// Needs to apply common block and execute tmat-dep code
		static boost::shared_ptr<OriAngleRes> calc(
			boost::shared_ptr<const OriTmatrix>, 
			double theta, double theta0,
			double phi, double phi0);
	};

	/// Class that takes the isotropic results and computes the Mueller 
	/// scattering matrix.
	class DLEXPORT_TMATRIX IsoAngleRes
	{
		public:
			IsoAngleRes();
			virtual ~IsoAngleRes();
			boost::shared_ptr<const OriTmatrix> tm;
			bool operator<(const IsoAngleRes &) const;
			std::map<double, std::array<double, 16> > P;

			static boost::shared_ptr<const IsoAngleRes> calc(
				boost::shared_ptr<const OriTmatrix>);
	};

	/// Get unpolarized differential backscatter cross-section.
	double DLEXPORT_TMATRIX getDifferentialBackscatterCrossSectionUnpol(
		boost::shared_ptr<const OriTmatrix> tm, bool flip = false);

	double DLEXPORT_TMATRIX getDifferentialBackscatterCrossSectionUnpol(
		boost::shared_ptr<const IsoAngleRes> angles);

	/// Calculate the projective area of an ellipsoid
	double DLEXPORT_TMATRIX calcProjArea(boost::shared_ptr<const tmatrixParams>,
		double beta);

	double DLEXPORT_TMATRIX calcProjAreaAvg(
		boost::shared_ptr<const tmatrixParams>);

	/// Calculate the surface area of an ellipsoid
	//double DLEXPORT_TMATRIX calcSA(boost::shared_ptr<const tmatrixParams>);

	/// Calculate the volume of an ellipsoid
	//double DLEXPORT_TMATRIX calcV(boost::shared_ptr<const tmatrixParams>);

	/// Print library debugging information
	void DLEXPORT_TMATRIX printDebugInfo(std::ostream &out = std::cerr);

	#pragma warning(pop)
}

