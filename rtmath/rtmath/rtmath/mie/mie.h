#pragma once
#include "../defs.h"

// boost provides the size-explicit data types for C++ compilers that
// lack full C99 support!
#include <boost/cstdint.hpp>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/strong_typedef.hpp>
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
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void DLEXPORT_RTMATH serialize(Archive &, rtmath::mie::mieParams &, const unsigned int);
		template <class Archive>
		void DLEXPORT_RTMATH serialize(Archive &, rtmath::mie::mieCalc &, const unsigned int);
		template <class Archive>
		void DLEXPORT_RTMATH serialize(Archive &, rtmath::mie::mieAngleRes &, const unsigned int);
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
		struct DLEXPORT_RTMATH mieBase
		{
			mieBase();
			double AXI, LAM, MRR, MRI, DDELT;
			bool operator<(const mieBase&) const;
		};

		class DLEXPORT_RTMATH mieParams
		{
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
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, mieParams &, const unsigned int);
		};

		class DLEXPORT_RTMATH mieCalc
		{
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
		private:
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, mieCalc &, const unsigned int);
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, ::rtmath::mie::mieAngleRes &, const unsigned int);
		};

		class DLEXPORT_RTMATH mieAngleRes
		{
		private:

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
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, mieAngleRes &, const unsigned int);
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, ::rtmath::mie::mieCalc &, const unsigned int);
		public:
			// Needs to apply common block and execute tmat-dep code
			static boost::shared_ptr<mieAngleRes> calc(
				boost::shared_ptr<const mieCalc>, 
				double theta);
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		// Function that gets unpolarized differential backscatter
		// cross-section.
		double DLEXPORT_RTMATH getDifferentialBackscatterCrossSectionUnpol(
			boost::shared_ptr<const mieCalc> om);

#pragma warning(pop)
	}
}


