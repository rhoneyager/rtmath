#pragma once
#include "../defs.h"
#include <complex>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/export.hpp>
#include "../Serialization/eigen_serialization.h"

namespace rtmath
{
	namespace ddscat
	{
		class ddScattMatrix;
		class ddScattMatrixF;
		class ddScattMatrixP;
	}
}

namespace rtmath
{
	namespace ddscat
	{
		enum class scattMatrixType
		{
			F,
			P
		};

		/**
		* This is now a base class because there are two ways for
		* specifying scattering: the complex scattering amplitude 
		* matrix or the scattering phase matrix. ddscat intermediate 
		* output gives the complex matrix which is more useful. The 
		* P matrix is harder to derive from, but is found in the 
		* .avg files and in tmatrix code.
		**/
		class DLEXPORT_rtmath_ddscat ddScattMatrix
		{
		public:
			typedef Eigen::Matrix4d PnnType;
			typedef Eigen::Matrix2cd FType;
			ddScattMatrix(double freq = 0, double theta = 0, double phi = 0, double thetan = 0, double phin = 0);
			virtual ~ddScattMatrix();
			//ddScattMatrix & operator = (const ddScattMatrix&); // Assignment needed due to arrays

			virtual PnnType mueller() const;
			inline double pol() const {return _pol;}
			inline void pol(double p) {_pol = p;}
			virtual scattMatrixType id() const { return scattMatrixType::P; }

			inline double freq() const { return _freq; }
			inline double theta() const { return _theta; }
			inline double thetan() const { return _thetan; }
			inline double phi() const { return _phi; }
			inline double phin() const { return _phin; }

			virtual bool operator<(const ddScattMatrix&) const;

			virtual bool compareTolHeader(const ddScattMatrix&, double tolPercent) const;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		protected:
			mutable PnnType _Pnn;
			//mutable boost::shared_ptr<matrixop> _Pnn;
			double _pol;
			double _freq, _theta, _thetan, _phi, _phin;
		private:
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		};

		/// \todo Move this into a lower-level header
		template <typename T>
		struct sharedComparator
		{
			bool operator()(const T &lhs, const T &rhs) const
			{
				//std::cerr << "Comparing " << lhs << "\t" << rhs << "\t";
				//bool a = (lhs < rhs);
				//std::cerr << a << "\t";
				bool res = (lhs)->operator<(*rhs);
				//std::cerr << res << std::endl;
				return res;
			}
		};

		class DLEXPORT_rtmath_ddscat ddScattMatrixF : public ddScattMatrix
		{
		public:
			/// Needs frequency (GHz) and phi (degrees) for P and K calculations
			ddScattMatrixF(double freq = 0, double theta = 0, double phi = 0, double thetan = 0, double phin = 0)
				: ddScattMatrix(freq, theta, phi, thetan, phin) {}
			virtual ~ddScattMatrixF();
			//ddScattMatrixF & operator = (const ddScattMatrixF&);
			virtual scattMatrixType id() const { return scattMatrixType::F; }
			// matrixop extinction() const;
			virtual PnnType mueller() const;
			void setF(const FType& fs);
			//void setF(std::istream &lss); // Include this higher up
			inline FType getF() const { return _f; }
			inline FType getS() const { return _s; }
		protected:
			void _calcS();
			void _calcP() const;
			mutable FType _f, _s;
			//boost::shared_array<std::complex<double> > _f, _s;
			//boost::shared_ptr<matrixop> _fRe, _fIm; // Should store as shared_array
			//boost::shared_ptr<matrixop> _sRe, _sIm;

			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		};

		/* class ddScattMatrixS : public ddScattMatrix
		{
		}; */

		class DLEXPORT_rtmath_ddscat ddScattMatrixP : public ddScattMatrix
		{
		public:
			ddScattMatrixP(double freq = 0, double theta = 0, double phi = 0, double thetan = 0, double phin = 0)
				: ddScattMatrix(freq, theta, phi, thetan, phin) {}
			virtual ~ddScattMatrixP() {}
			//ddScattMatrixP & operator = (const ddScattMatrixP&);
			virtual scattMatrixType id() const { return scattMatrixType::P; }
			inline void setP(const PnnType& v) { _Pnn = v; }
			PnnType getP() const { return _Pnn; }

			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		};
	}
}


BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddScattMatrix);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddScattMatrixF);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddScattMatrixP);

