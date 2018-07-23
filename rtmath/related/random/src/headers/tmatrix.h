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
namespace tmatrix_random
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
namespace tmatrix_random
{
	/// Base structure for holding T-matrix inputs
	struct DLEXPORT_TMATRIX_RANDOM tmatrixBase
	{
		tmatrixBase();
		double AXI, RAT, LAM, GAM,
			MRR, MRI, EPS,
			DDELT, R1, R2, AXMAX, B;
		boost::int32_t NP, NDGS, NDISTR, NPNAX, NPNA, NKMAX;
		bool operator<(const tmatrixBase&) const;
	};

	/// Class that sets up the Fortran code for a run with T-matrix inputs.
	class DLEXPORT_TMATRIX_RANDOM tmatrixParams
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
			boost::int32_t ndgs /// number or integral division points
			);
		double axi, rat, lam, gam,
			eps, ddelt, r1, r2, axmax, b;
		std::complex<double> m; /// Refractive index
		boost::int32_t np, ndgs, ndistr, npnax, npna, nkmax, dbg;
		tmatrixParams();
		bool operator==(const tmatrixParams&) const;
		bool operator<(const tmatrixParams&) const;
	private:
		bool needApply() const;
		void apply() const;
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
	class DLEXPORT_TMATRIX_RANDOM OriTmatrix
	{
	public:
		OriTmatrix();
		virtual ~OriTmatrix() {}
		// Need to apply and execute tmat code
		static boost::shared_ptr<const OriTmatrix> calc(
			boost::shared_ptr<const tmatrixParams> in);
		static boost::shared_ptr<const OriTmatrix> calc(
			const tmatrixBase&);
		boost::shared_ptr<const tmatrixParams> base;
		// Quantities imported from Mischenko's code
		double qsca; /// Rotation-independent total scattering cross-section
		double qext; /// Rotation-independent total extinction cross-section
		double walb; /// Albedo
		double g; // asymmetry parameter <cos theta>
		double qbk; /// Backscatter cross-section
		double time; /// Computation time
		int nmax;
		boost::shared_ptr<isoRes> iso_angle_res;
	private:
		void import(); /// Import from Mischenko Fortran code
		void calcParams(); /// Calculate my parameters
	};

	/// Class that takes the isotropic results and computes the Mueller 
	/// scattering matrix.
	class DLEXPORT_TMATRIX_RANDOM IsoAngleRes
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

	double DLEXPORT_TMATRIX_RANDOM getDifferentialBackscatterCrossSectionUnpol(
		boost::shared_ptr<const IsoAngleRes> angles);

	/// Print library debugging information
	void DLEXPORT_TMATRIX_RANDOM printDebugInfo(std::ostream &out = std::cerr);

	#pragma warning(pop)
}

