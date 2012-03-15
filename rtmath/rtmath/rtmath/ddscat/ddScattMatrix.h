#pragma once
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../matrixop.h"
#include "../phaseFunc.h"
#include "../coords.h"

namespace rtmath {
	namespace ddscat {
		class ddScattMatrix
		{
			// A set of complex matrices containing the
			// 2x2 complex scattering amplitudes and the
			// extinction and scattering matrices
		public:
			ddScattMatrix(double freq, double theta, double phi);
			ddScattMatrix(double freq, const std::string &lin);
			ddScattMatrix(double freq, std::istream &lss);
			ddScattMatrix();
			~ddScattMatrix();
			ddScattMatrix & operator = (const ddScattMatrix&); // Assignment needed due to double arrays

			inline double theta() const {return _theta;}
			inline double phi() const {return _phi;}
			inline double freq() const {return _freq; }
			
			void setF(const std::complex<double> fs[2][2]);
			void setF(const matrixop &src); // Double-sized matrixop for complexity
			void setF(size_t i, size_t j, const std::complex<double> &val);
			void setF(const std::string &lin);
			void setF(std::istream &lss);

			void getF(matrixop &res) const; // Double-sized matrixop for complexity

			void genCoords(coords::cyclic<double> &res) const;
			inline coords::cyclic<double> genCoords() const { coords::cyclic<double> res; genCoords(res); return res; }

			void print(std::ostream &out = std::cerr) const;
			void writeCSV(const std::string &filename) const;
			void writeCSV(std::ostream &out = std::cerr) const;

			void mueller(double Pnn[4][4]) const;
			void mueller(matrixop &res) const;
			inline matrixop mueller() const { matrixop res(2,4,4); mueller(res); return res; }
			void extinction(double Knn[4][4]) const;
			void extinction(matrixop &res) const;
			inline matrixop extinction() const { matrixop res(2,4,4); extinction(res); return res; }
		private:
			void _init();
			void _genS();
			
			double _theta, _phi, _wavelength, _freq;
			double _Knn[4][4], _Pnn[4][4];
			// Unfortunately, matrixop is not a template. Otherwise, I'd change these.
			std::complex<double> _vals[2][2];
			std::complex<double> _S[4];
		};
	}
}

// ostream override
std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddScattMatrix &ob);
// istream override
std::istream &operator>>(std::istream &stream, rtmath::ddscat::ddScattMatrix &ob);
