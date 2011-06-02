#pragma once

#include "../rtmath-base/phaseFunc.h"
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
#include "../rtmath-base/enums.h"
#include "damatrix.h"
#include <map>
#include <vector>

namespace rtmath {

	class dalayer
	{
	public:
		/* Any layer takes on some parameters
		* These include optical depth (tau) at zenith,
		* The type of layer (from a derived class),
		* a generating function, 
		* single-scattering albedo,
		* a base phase function,
		* etc.
		* Other parameters (mu, mun, phi, phin) are used in layer 
		* calculation and are calculated repeatedly for each angle
		* (TODO: cache these)
		*/
		dalayer(matrixop &pf, double alb);
		virtual ~dalayer(void);
		void tau(double newtau) {_tau = newtau;}
		double tau() {return _tau;}
		void generateLayer();
		// TODO: redefine calcR and calcT as aliases / macros of 
		//    calcParam
		/*
		inline damatrix calcR(double tau, double phi, double phin, 
		double mu, double mun);
		inline damatrix calcT(double tau, double phi, double phin, 
		double mu, double mun);
		damatrix calcParam(double tau, double phi, double phin, 
		double mu, double mun, rtselec::rtselec rt);
		*/
	protected:
		double _tau;
		matrixop *_pf;
		double _ssa;
	};

	class dalayer_init : public damatrix
	{
	public:
		dalayer_init(matrixop &pf, double alb, double tau, rtselec::rtselec rt) : damatrix(pf.size())
		{
			_phaseMat = pf;
			_tau = tau;
			_ssa  = alb;
			_rt = rt;
		}
		virtual ~dalayer_init();
		virtual boost::shared_ptr<damatrix> eval(const mapid &valmap);
	private:
		matrixop _phaseMat;
		double _ssa;
		double _tau;
		rtselec::rtselec _rt;
	};

}; // end rtmath


