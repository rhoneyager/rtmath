#pragma once

#include <string>
#include <set>
#include <boost/shared_ptr.hpp>

// Forward declaration for boost::serialization below
namespace rtmath {
	class matrixop;
	namespace ddscat {
		class rotationsBase;
		class rotations;
	}
}

// Need these so the template friends can work
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive &, rtmath::ddscat::rotationsBase &, const unsigned int);

		template <class Archive>
		void serialize(Archive &, rtmath::ddscat::rotations &, const unsigned int);
	}
}

namespace rtmath {
	namespace ddscat {
		class ddPar;

		class rotationsBase
		{
		public:
			rotationsBase();
			virtual ~rotationsBase();
		protected:
			double _bMin, _bMax;
			double _tMin, _tMax;
			double _pMin, _pMax;
			size_t _bN, _tN, _pN;

			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, rotationsBase &, const unsigned int);
		};

		class rotations : public rotationsBase
		{
		public:
			rotations();
			rotations(const ddPar &src);
			rotations(double bMin, double bMax, size_t bN,
				double tMin, double tMax, size_t tN,
				double pMin, double pMax, size_t pN);
			static boost::shared_ptr<rotations> create(
				double bMin, double bMax, size_t bN,
				double tMin, double tMax, size_t tN,
				double pMin, double pMax, size_t pN);
			static boost::shared_ptr<rotations> create();
			static boost::shared_ptr<rotations> create(const ddPar &src);
			virtual ~rotations();
			double bMin() const { return _bMin; }
			double bMax() const { return _bMax; }
			size_t bN() const { return _bN; }
			double tMin() const { return _tMin; }
			double tMax() const { return _tMax; }
			size_t tN() const { return _tN; }
			double pMin() const { return _pMin; }
			double pMax() const { return _pMax; }
			size_t pN() const { return _pN; }
			// ddPar output function
			void out(ddPar &dest) const;
			void betas(std::string &dest) const;
			void thetas(std::string &dest) const;
			void phis(std::string &dest) const;
			void betas(std::set<double> &b) const;
			void thetas(std::set<double> &t) const;
			void phis(std::set<double> &p) const;

			bool operator==(const rotations &rhs) const;
			bool operator!=(const rotations &rhs) const;
			bool operator<(const rotations &rhs) const;

			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, rotations &, const unsigned int);
		};

		// Function to calculate Gimbal matrices and effective rotation matrix.
		// Using ddscat conventions.
		void rotationMatrix(double thetad, double phid, double betad,
			matrixop &Rx, matrixop &Ry, matrixop &Rz, matrixop &Reff);
		void rotationMatrix(double thetad, double phid, double betad,
			matrixop &Reff);

	}
}
