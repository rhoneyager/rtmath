#pragma once

// This is the header for the initial layer generation class.
// This is used when creating a doubling-adding layer from scratch, and this 
// provides the fundamental, infinitesimally small initial layer.
// It derives from the standard damatrix, and overrides the constructor and eval.

#include <memory>
#include "matrixop.h"
#include "damatrix.h"
#include "enums.h" // provides rtselec

namespace rtmath {

	class daInitLayer : public damatrix
	{
	public:
		daInitLayer(std::shared_ptr<matrixop> pf, double alb, double tau, rtselec::rtselec rt);
		daInitLayer(std::shared_ptr<damatrix> pf, double alb, double tau, rtselec::rtselec rt);
		virtual ~daInitLayer();
		virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
	protected:
		std::shared_ptr<damatrix> _phaseMatRot;
		double _ssa;
		double _tau;
		rtselec::rtselec _rt;
	private:
		void _init(std::shared_ptr<damatrix> pf, 
			double alb, double tau, rtselec::rtselec rt);
	};

}; // end rtmath


