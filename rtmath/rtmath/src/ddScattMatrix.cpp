#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"

namespace rtmath {
	namespace ddscat {
		void ddScattMatrix::_init()
		{
			_theta = 0;
			_phi = 0;
			_freq = 0;
			_wavelength = 0;
			for (size_t i=0;i<4;i++)
				for (size_t j=0;j<4;j++)
				{
					_Knn[i][j] = 0;
					_Pnn[i][j] = 0;
				}
			// _vals and _S are part of std::complex.
			// They are automatically zeroed.
		}
		
		ddScattMatrix::ddScattMatrix()
		{
			_init();
		}
		
		ddScattMatrix::~ddScattMatrix()
		{
		}
		
		ddScattMatrix::ddScattMatrix(double freq, double theta, double phi)
		{
			_init();
			_theta = theta;
			_phi = phi;
			_freq = freq;
			// freq is in GHz. Convert to wavelength in um.
			units::conv_spec ftowv("GHz","um");
			_wavelength = ftowv.convert(freq);
		}
		
		ddScattMatrix::ddScattMatrix(double freq, const std::string &lin)
		{
			_init();
			_freq = freq;
			units::conv_spec ftowv("GHz","um");
			_wavelength = ftowv.convert(freq);
			setF(lin);
		}

		ddScattMatrix::ddScattMatrix(double freq, std::istream &lss)
		{
			_init();
			_freq = freq;
			units::conv_spec ftowv("GHz","um");
			_wavelength = ftowv.convert(freq);
			setF(lss);
		}
		
		ddScattMatrix & ddScattMatrix::operator=(const ddScattMatrix &rhs)
		{
			if (this == &rhs) return *this; // self-assignment check

			_theta = rhs.theta();
			_phi = rhs.phi();
			_freq = rhs.freq();
			_wavelength = rhs._wavelength;

			for (size_t i=0;i<2;i++)
				for (size_t j=0;j<2;j++)
				{
					_vals[i][j] = rhs._vals[i][j];
				}
			_genS();

			return *this;
		}

		void ddScattMatrix::mueller(double Pnn[4][4]) const
		{
			for (size_t i=0;i<4;i++)
				for (size_t j=0;j<4;j++)
				{
					Pnn[i][j] = _Pnn[i][j];
				}
		}
		
		void ddScattMatrix::extinction(double Knn[4][4]) const
		{
			for (size_t i=0;i<4;i++)
				for (size_t j=0;j<4;j++)
			{
				Knn[i][j] = _Knn[i][j];
			}
		}
		
		void ddScattMatrix::mueller(matrixop &res) const
		{
			res.resize(2,4,4);
			res.fromDoubleArray(&_Pnn[0][0]);
		}
		
		void ddScattMatrix::extinction(matrixop &res) const
		{
			res.resize(2,4,4);
			res.fromDoubleArray(&_Knn[0][0]);
		}
		
		void ddScattMatrix::genCoords(coords::cyclic<double> &res) const
		{
			coords::cyclic<double> ret(3,freq(),theta(),phi());
			res = ret;
		}

		void ddScattMatrix::setF(size_t i, size_t j, const std::complex<double> &val)
		{
			if (i>1 || j>1) throw debug::xArrayOutOfBounds();
			_vals[i][j] = val;
			_genS();
		}
		
		void ddScattMatrix::setF(const std::complex<double> fs[2][2])
		{
			for (size_t i=0;i<2;i++)
				for (size_t j=0;j<2;j++)
				{
					_vals[i][j] = fs[i][j];
				}
			_genS();
		}
		
		void ddScattMatrix::setF(std::istream &lss)
		{
			// This function reads directly from a string and extracts the appropriate values.
			// It compartamentalizes ddscat .fml reads into the appropriate classes.
			// May be called from public function or by constructor.
			// Called by istream operator.
			using namespace std; 
			double re, im;
			//istringstream lss(lin);
			lss >> _theta >> _phi;
			for (size_t i=0;i<4;i++)
			{
				lss >> re >> im;
				complex<double> nval(re,im);
				size_t j=i%2;
				size_t k=i/2;
				_vals[j][k] = nval;
			}
			
			_genS();
		}
		
		void ddScattMatrix::setF(const std::string &lin)
		{
			using namespace std;
			istringstream lss(lin);
			setF(lss);
		}
		
		void ddScattMatrix::_genS()
		{
			// Generates Snn and, by extension, Knn and Pnn
			// TODO: verify Snn, Pnn and Knn
			using namespace std;
			//complex<double> S[4];
			complex<double> i(0,1);

			complex<double> e01x(0,0), e01y(1,0), e01z(0,0), e02x(0,0), e02y(0,0), e02z(1,0);
			complex<double> a = conj(e01y), b=conj(e01z), c=conj(e02y), d=conj(e02z);

			//double cp = cos(2.0*M_PI*phi()/180.0);
			double cp = cos(_phi * M_PI / 180.0);
			//double sp = sin(2.0*M_PI*phi()/180.0);
			double sp = sin(_phi * M_PI / 180.0);
			_S[0] = -i * ( _vals[1][0] * (b * cp - a * sp) + _vals[1][1] * (d * cp - c * sp) );
			_S[1] = -i * ( _vals[0][0] * (a*cp + b * sp) + _vals[0][1] * (c * cp + d * sp) );
			_S[2] = i * ( _vals[0][0] * (b * cp - a * sp) + _vals[0][1] * (d * cp - c * sp) );
			_S[3] = i * ( _vals[1][0] * (a*cp + b * sp) + _vals[1][1] * (c*cp + d * sp) );
			
			rtmath::scattMatrix::_genExtinctionMatrix(_Knn, _S, _freq);
			rtmath::scattMatrix::_genMuellerMatrix(_Pnn,_S);
		}

		void ddScattMatrix::setF(const matrixop &src)
		{
			_vals[0][0].real(src.get(2,0,0));
			_vals[0][0].imag(src.get(2,0,1));
			_vals[0][1].real(src.get(2,0,2));
			_vals[0][1].imag(src.get(2,0,3));
			_vals[1][0].real(src.get(2,1,0));
			_vals[1][0].imag(src.get(2,1,1));
			_vals[1][1].real(src.get(2,1,2));
			_vals[1][1].imag(src.get(2,1,3));
			_genS();
		}

		void ddScattMatrix::getF(matrixop &src) const
		{
			src.resize(2,2,4);
			src.set(_vals[0][0].real(),2,0,0);
			src.set(_vals[0][0].imag(),2,0,1);
			src.set(_vals[0][1].real(),2,0,2);
			src.set(_vals[0][1].imag(),2,0,3);
			src.set(_vals[1][0].real(),2,1,0);
			src.set(_vals[1][0].imag(),2,1,1);
			src.set(_vals[1][1].real(),2,1,2);
			src.set(_vals[1][1].imag(),2,1,3);
		}
		
		void ddScattMatrix::print(std::ostream &out) const
		{
			using namespace std;
			out << "ddScattMatrix for theta " << _theta << " phi " << _phi
					<< " frequency " << _freq << endl;
			
			out << "f" << endl;
			for (size_t i=0; i<2; i++)
				for (size_t j=0; j<2; j++)
					out << i << "," << j << "\t" << _vals[i][j] << endl;
			out << "S" << endl;
			for (size_t i=0; i<4; i++)
			{
				out << "\t" <<  _S[i] << endl;
			}

			out << "Mueller" << endl;
			out << _theta << "\t" << _phi << "\t" << _freq << "\t";
			//update();
			for (size_t i=0; i<4; i++)
			{
				for (size_t j=0; j<4; j++)
				{
					out << _Pnn[i][j] << "\t";
				}
				out << endl;
			}
			out << endl;

			out << "Extinction" << endl;
			for (size_t i=0; i<4; i++)
			{
				for (size_t j=0; j<4; j++)
				{
					out << _Knn[i][j] << "\t";
				}
				out << endl;
			}
			out << endl;
		}
		
		void ddScattMatrix::writeCSV(std::ostream &out) const
		{
			using namespace std;
			out << _freq << ", " << _theta << ", " << _phi << ", ";
			/*
			for (size_t i=0; i<2; i++)
			{
				for (size_t j=0; j<2; j++)
				{
					out << _vals[i][j].real() << ", " << _vals[i][j].imag() << ", ";
				}
			}
			*/
			for (size_t i=0; i<4; i++)
			{
				for (size_t j=0; j<4; j++)
				{
					out << _Pnn[i][j] << ", ";
				}
			}

			for (size_t i=0; i<4; i++)
			{
				for (size_t j=0; j<4; j++)
				{
					out << _Knn[i][j] << ", ";
				}
			}
			out << endl;
		}
		
		void ddScattMatrix::writeCSV(const std::string &filename) const
		{
			std::ofstream out(filename.c_str());
			out << "CSV output for (frequency,theta,phi)\n";
			writeCSV(out);
		}
		
		
	} // end ddscat
} // end rtmath

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddScattMatrix &ob)
{
	ob.writeCSV(stream);
	return stream;
}

std::istream &operator>>(std::istream &stream, rtmath::ddscat::ddScattMatrix &ob)
{
	ob.setF(stream);
	return stream;
}


