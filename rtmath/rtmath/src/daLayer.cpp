#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"
#include "../rtmath/da/daDiagonalMatrix.h"
#include <memory>

namespace rtmath {

	daLayer::daLayer(std::shared_ptr<damatrix> pf, double tau, double alb)
	{
		_tau = tau;
		_pf = pf;
		_layergenerated = false;
		_alb = alb;
		generateLayer();
	}

	daLayer::~daLayer()
	{
	}
	
	double daLayer::tau()
	{
		return _tau;
	}

	void daLayer::tau(double newtau)
	{
		_layergenerated = false;
		_tau = newtau;
		R.reset();
		T.reset();
		generateLayer();
	}

	void daLayer::generateLayer()
	{
		if (_layergenerated) return;
		using namespace std;

		// Calculate the number of doubles required to make a complete layer
		unsigned int numDoubles = 0;
		double taueff = _tau;
		do {
			taueff /= 2.0;
			numDoubles++;
		} while (taueff >= 1e-10);

		std::cout << numDoubles << endl;
		// Construct the initial layers
		// Any reflections to get R or T depend on the source of the pf.
		// Reflections are handled in the daInitLayer code itself.
		std::shared_ptr<daInitLayer> _Rinit(new daInitLayer(_pf, _alb, _tau, rtselec::R));
		std::shared_ptr<daInitLayer> _Tinit(new daInitLayer(_pf, _alb, _tau, rtselec::T));
		
		// Cast the initial layers as damatrices
		std::shared_ptr<damatrix> Rc = static_pointer_cast<damatrix>(_Rinit);
		std::shared_ptr<damatrix> Tc = static_pointer_cast<damatrix>(_Tinit);

		std::shared_ptr<damatrix> D;	// Internal Downwelling
		std::shared_ptr<damatrix> U;	// Internal Upwelling
		std::shared_ptr<damatrix> S;	// Used to get D
		std::shared_ptr<damatrix> Qi;	// Calculate Q1
		std::shared_ptr<damatrix> Qc;	// Running multiplication to get Q^n 
		std::shared_ptr<damatrix> RcD;	// Doubled layer reflectance
		std::shared_ptr<damatrix> TcD;	// Doubled layer transmittance
		while (numDoubles > 0)
		{
			// Double the layer

			// First, calculate Q1
			Qi = damatrix::op(Rc,Rc,MULT);
			S = Qi;
			Qc = Qi;
			// Then, recursively multiply to get Qn for a large n, say 10
			for (unsigned int i=0;i<10;i++)
			{
				// I'm doing this to get away with not implementing powers in my matrices
				Qc = damatrix::op(Qc,Qi,MULT);
				S = damatrix::op(S,Qc,ADD);
			}
			
			// S has been calculated. Now to find D, U, RcD and TcD
			// Constrict the diagonal e^-tau/mu and mu0 matrices for this doubling
			std::shared_ptr<daDiagonalMatrix> dMu(new daDiagonalMatrix(taueff, valmap_selector::MU));
			std::shared_ptr<daDiagonalMatrix> dMun(new daDiagonalMatrix(taueff, valmap_selector::MUN));
			std::shared_ptr<damatrix> dMuD = static_pointer_cast<damatrix>(dMu);
			std::shared_ptr<damatrix> dMunD = static_pointer_cast<damatrix>(dMun);

			std::shared_ptr<damatrix> D_a = damatrix::op(S,Tc,MULT);
			std::shared_ptr<damatrix> D_b = damatrix::op(S,dMunD,MULTNORMAL);
			std::shared_ptr<damatrix> D_c = damatrix::op(D_a,Tc,ADD);
			D = damatrix::op(D_c,D_b,ADD);
			// D has been found. This notation unfortunately cannot be simplified.

			std::shared_ptr<damatrix> U_a = damatrix::op(Rc,D,MULT);
			std::shared_ptr<damatrix> U_b = damatrix::op(Rc,dMunD,MULTNORMAL);
			U = damatrix::op(U_a,U_b,ADD);
			// U has been found.

			std::shared_ptr<damatrix> R_a = damatrix::op(dMuD,U,MULTNORMAL);
			std::shared_ptr<damatrix> R_b = damatrix::op(Tc,U,MULT);
			std::shared_ptr<damatrix> R_c = damatrix::op(R_a,R_b,ADD);
			RcD = damatrix::op(Rc,R_c,ADD);
			// RcD (the doubled layer's reflectance) has been found

			std::shared_ptr<damatrix> T_a = damatrix::op(Tc,D,MULT);
			std::shared_ptr<damatrix> T_b = damatrix::op(Tc,dMunD,MULTNORMAL);
			std::shared_ptr<damatrix> T_c = damatrix::op(dMunD,D,MULTNORMAL);
			std::shared_ptr<damatrix> T_d = damatrix::op(T_a,T_b,ADD);
			TcD = damatrix::op(T_d,T_c,ADD);
			// TcD (the doubled layer's diffuse transmission) has been found


			// Prepare for the next iteration of the loop
			Tc = TcD;
			Rc = RcD;
			numDoubles--;
			taueff *= 2;
		}

		// Set the final values for the matrices here
		T = Tc;
		R = Rc;
		_layergenerated = true; // Don't run this function again for this layer
	}

}; // end namespace rtmath

