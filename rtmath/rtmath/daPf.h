#pragma once

// This is the header for the damatrix-derived phasefunction class.
// This is better than phaseFunc of phaseFuncRotator because it 
// uses damatrix to provide a good structure and is only evaled some of 
// the time. It is sensitive to R and T differences, and can be 
// constructed from a P(alpha) matrix or P(mu,mun,phi,phin).

#include <memory>
#include "matrixop.h"
#include "damatrix.h"
#include "phaseFunc.h"
#include "enums.h" // provides rtselec

// Will provide two phase function matrices, both deriving from daPf.
// First is just the normal phase function that takes a mapid
// (mu,mun,phi,phin). The second takes a phase function that is dependent 
// on the total scattering angle, alpha. It transforms and rotates P(alpha)
// to give the appropriate phase function for further computation.

namespace rtmath {
	// No scattering at all. Just return the basic matrix.
	class daPfNone : public damatrix
	{
	public:
		daPfNone();
		virtual ~daPfNone() {}
		virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
	private:
		std::shared_ptr<matrixop> _pf;
	};

	// daPfAlpha is for phase functions that depend on alpha
	// It provides P_t=P(mu,mu0,phi,phi0). Reflection is handled with a 
	// converter, which uses symmetry to change the arguments.
	class daPfAlpha : public damatrix
	{
	public:
		daPfAlpha(std::shared_ptr<phaseFunc> pf);
		virtual ~daPfAlpha();
		virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
	protected:
		std::shared_ptr<phaseFunc> _phaseMat;
		std::shared_ptr<damatrix> _lhs, _rhs;
		rtselec::rtselec _rt;
		// Inherited from damatrix:
		//mutable std::map<mapid, std::shared_ptr<matrixop>, mmapcomp > _eval_cache;
	private:
		void _init(std::shared_ptr<phaseFunc> pf);
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
			friend class rtmath::daPfAlpha;
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


	// This namespace contains the necessary functions to exploit symmetry on a damatrix.
	// Useful especially for phasefunctions, or anything using T, T*, R and R*.
	namespace daPfReflections
	{
		enum daReflectionEnum
		{
			T,
			R,
			T_STAR,
			R_STAR
		};

		class daReflection : public damatrix
		{
		public:
			daReflection(daReflectionEnum spec, std::shared_ptr<rtmath::damatrix> pf);
			virtual ~daReflection() {}
			virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
		protected:
			daReflectionEnum _type;
			std::shared_ptr<rtmath::damatrix> _source;
		};

	}; // end namespace daPfReflections
}; // end rtmath


