#pragma once
#include "../defs.h"

// boost provides the size-explicit data types for C++ compilers that
// lack full C99 support!
#include <boost/cstdint.hpp>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/strong_typedef.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/access.hpp>
#include <set>
#include <map>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

// Need these so the template friends can work
namespace rtmath
{
	namespace mie
	{
		struct mieBase;
		class mieParams;
		class mieCalc;
		class mieAngleRes;
	}
}

// Redefine double to allow serialization of shared_ptr<vector<double> >
#ifndef EXT_TYPES
#define EXT_TYPES
BOOST_STRONG_TYPEDEF(double, tDouble);
BOOST_STRONG_TYPEDEF(boost::int32_t, tREAL);
#endif

#pragma warning(push)
#pragma warning(disable: 4251)

namespace rtmath
{
	namespace mie
	{
		struct DLEXPORT_rtmath_mie mieBase
		{
			mieBase();
			double AXI, LAM, MRR, MRI, DDELT;
			bool operator<(const mieBase&) const;
		};

		class DLEXPORT_rtmath_mie mieParams
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			virtual ~mieParams() {}
			static boost::shared_ptr<const mieParams> create(const mieBase&);
			static boost::shared_ptr<const mieParams> create(
				double axi, // equiv sphere radius
				double lam, // wavelength of incident light
				double mrr, // real refractive index
				double mri, // imaginary refractive index (positive)
				double ddelt = 0.001 // computational accuracy
				);
			double axi, lam, ddelt;
			std::complex<double> m;
			mieParams();
		private:
			friend class mieCalc;
			friend class mieAngleRes;
		};

		class DLEXPORT_rtmath_mie mieCalc
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			mieCalc();
			virtual ~mieCalc() {}
			// Need to apply and execute tmat code
			static boost::shared_ptr<const mieCalc> calc(
				boost::shared_ptr<const mieParams> in);
			static boost::shared_ptr<const mieCalc> calc(const mieBase&);
			boost::shared_ptr<const mieParams> base;
			double qsca;
			double qext;
			double qabs;
			double qbk;
			double g;
			double walb;
			double sizep;
		};

		class DLEXPORT_rtmath_mie mieAngleRes
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			mieAngleRes();
			boost::shared_ptr<const mieCalc> mc;
			double theta;
			double mu;

			std::complex<double> getS(size_t i, size_t j) const;
			double getSnn(size_t i, size_t j) const;
			double getP(size_t i, size_t j) const;

			bool operator<(const mieAngleRes &rhs) const;
		private:
			Eigen::Matrix2cd S;
			Eigen::Matrix4d P;
			//boost::shared_ptr< const std::vector<std::complex<tDouble> > > S;
			//boost::shared_ptr< const std::vector<tDouble> > P;
		public:
			// Needs to apply common block and execute tmat-dep code
			static boost::shared_ptr<mieAngleRes> calc(
				boost::shared_ptr<const mieCalc>, 
				double theta);
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		// Function that gets unpolarized differential backscatter
		// cross-section.
		double DLEXPORT_rtmath_mie getDifferentialBackscatterCrossSectionUnpol(
			boost::shared_ptr<const mieCalc> om);

#pragma warning(pop)
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::mie::mieParams);
BOOST_CLASS_EXPORT_KEY(rtmath::mie::mieCalc);
BOOST_CLASS_EXPORT_KEY(rtmath::mie::mieAngleRes);

