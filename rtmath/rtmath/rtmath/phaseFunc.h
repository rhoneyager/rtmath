#pragma once
#include "defs.h"

#include <functional>
#include <complex>
#include <map>
#include <string>
#include <Eigen/Core>
//#include "enums.h"
//#include "da/damatrix.h"


#pragma message("Warning: phaseFunc.h needs the pf class moved elsewhere, + interpolation")
namespace rtmath {

	/** \brief This namespace provides the different type of radiative 
	* transfer matrix manipulations.
	* 
	* This includes several Mueller matrix generation methods and 
	* the ability to generate an extinction matrix. Eventually, Mueller matrix 
	* inversion routines will also go here.
	*
	* \todo Need to move pf class elsewhere and add interpolation
	**/
	namespace phaseFuncs
	{
		/// Provides a reference to the desired phase function 
		/// routine. This is implemented to allow user choice in Mueller method.
		void DLEXPORT_rtmath_core selectMueller(const std::string &id,
			std::function<void(const Eigen::Matrix2cd&, Eigen::Matrix4d&)>&);

		// Note following conventions: matrix is [[S2, S3][S4,S1]] = [[Sn0, Sn1][Sn2, Sn3]]
		// Sn is the matrix in linear form {S1, S2, S3, S4}, so it should avoid any 
		// of the subsequent issues with forgetting the index transformations.

		void DLEXPORT_rtmath_core muellerBH(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn);
		void DLEXPORT_rtmath_core muellerTMATRIX(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn);

		void DLEXPORT_rtmath_core convertFtoS(const Eigen::Matrix2cd &f, Eigen::Matrix2cd& Sn, double phi, 
			std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d);

		void DLEXPORT_rtmath_core invertS(const Eigen::Matrix4d &Snn, const Eigen::Matrix4d &Knn, double fGHz, Eigen::Matrix2cd& Sn);

		void DLEXPORT_rtmath_core genExtinctionMatrix(Eigen::Matrix4d &Knn, const Eigen::Matrix2cd &Sn, double fGHz);

	}

	/* // These will be reimplemented by the da code?
	class phaseFunc // TODO: rewrite this.
	{
	public:
		// This fuction is designed to be pure virtual, so that mie and other
		// phase functions can be included in atmospheric layers
		phaseFunc(void) {}
		virtual ~phaseFunc(void) {}
		// eval evaluates P for reduced mur and phir (mu-mu0, phi-phi0) to get proper directional data
		//virtual void eval(double mu, double mun, double phir, double Pnn[4][4], double res[4][4]);
		// calc constructs the base P matrix, centered on mu=mu0
		//virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]) = 0;
		//virtual std::shared_ptr<matrixop> eval(double alpha) const = 0;
	protected:
		//mutable std::map<double,std::shared_ptr<matrixop> > _eval_cache;
	};

	class scattMatrix
	{
	public:
		scattMatrix(void) {}
		virtual ~scattMatrix(void) {}
		// Calculate the scattering amplitude matrix for a given mu.
		// Here, mu = cos(alpha), the total scattering angle.
		virtual void calc(double mu, double Snn[4][4], std::complex<double> Sn[4]) = 0;
	public:
		// f is the frequency
		//static void _genMuellerMatrix(double Snn[4][4], const std::complex<double> Sn[4]);

	};
	*/
}
