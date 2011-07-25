#pragma once

// This is the header for the damatrix-derived phasefunction class.
// This is better than phaseFunc of phaseFuncRotator because it 
// uses damatrix to provide a good structure and is only evaled some of 
// the time. It is sensitive to R and T differences, and can be 
// constructed from a P(alpha) matrix or P(mu,mun,phi,phin).

#include <memory>
#include "matrixop.h"
#include "damatrix.h"
#include "enums.h" // provides rtselec

namespace rtmath {
	class daPf : public damatrix, rtmath::debug::defective
	{
	public:
		daPf(rtselec::rtselec RT, std::shared_ptr<damatrix> pf);
		daPf(rtselec::rtselec RT, std::shared_ptr<matrixop> pf);
		virtual ~daPf();
		virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
	protected:
		std::shared_ptr<damatrix> _phaseMat;
		rtselec::rtselec _rt;
		std::shared_ptr<damatrix> _lhs, _rhs;
		// Inherited from damatrix:
		//mutable std::map<mapid, std::shared_ptr<matrixop>, mmapcomp > _eval_cache;
	private:
		void _init(rtselec::rtselec RT, std::shared_ptr<damatrix> pf);
	};


	namespace daPfRotators {

		enum daRotatorLR
		{
			LHS,
			RHS
		};

		class daRotator : public damatrix
		{
			// I want to keep this class hidden, so only daPf can initialize it as a member.
			friend class rtmath::daPf;
		protected:
			daRotator(rtselec::rtselec RT, daRotatorLR LR);
		public:
			virtual ~daRotator() {}
			virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
		protected:
			rtselec::rtselec _rt;
			daRotatorLR _lr;
		};
	}; // end namespace daPfRotators
}; // end rtmath


