#pragma once

#include "../rtmath-base/phaseFunc.h"
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
#include "../rtmath-base/enums.h"
#include "lbl.h"
#include "damatrix.h"
#include "debug.h"
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>

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
		dalayer(); // empty dalayer for no-scattering case
		virtual ~dalayer(void);
		void tau(double newtau) {_tau = newtau;}
		double tau() {return _tau;}
		void generateLayer();
		boost::shared_ptr<damatrix> getR() { return _R; }
		boost::shared_ptr<damatrix> getT() { return _T; }
		boost::shared_ptr<damatrix> getU() { return _U; }
		boost::shared_ptr<damatrix> getD() { return _D; }
		//std::set<lbl::specline*> lines; // The lines that matter in the layer
	protected:
		double _tau;
		matrixop *_pf;
		double _ssa;
		boost::shared_ptr<damatrix> _R, _T, _U, _D;
	};

	
	class dalayerInit : public damatrix
	{
	public:
		dalayerInit(matrixop &pf, double alb, double tau, rtselec::rtselec rt) : damatrix(pf.size())
		{
			_phaseMat = &pf;
			_tau = tau;
			_ssa  = alb;
			_rt = rt;
		}
		virtual ~dalayerInit();
		virtual boost::shared_ptr<damatrix> eval(const mapid &valmap);
	private:
		matrixop *_phaseMat;
		double _ssa;
		double _tau;
		rtselec::rtselec _rt;
	};
	
	/*
	class layer {
		// This class contains the actual atmospheric layer (not just the DA layer components)
	public:
		layer() {}
		~layer() {}
		boost::shared_ptr<dalayer> l_DA;
		boost::shared_ptr<lbllayer> l_lbl;
	protected:
	private:
	};
	*/

}; // end rtmath


