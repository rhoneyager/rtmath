#pragma once

// The damatrix is the fundamental unit for doubling-adding calculations
#include <memory>
#include <map>
#include <unordered_map>
//#pragma message("d-3")
#include <boost/unordered/unordered_map.hpp> // Instead of std::unordered map, for now
#include <limits.h>	
#include "../enums.h"
#include "../mapid.h"
#include "../matrixop.h"
#include "../defs.h"
#include "damatrix_quad.h"
//#include "../error/error.h"

namespace rtmath {
	
	enum daOp
	{
		NONE,
		ADD,
		MULT,
		MULTNORMAL,
		INV,
		POW
	};

	class damatrix
	{
	protected:
		damatrix();
	public:
		damatrix(const std::shared_ptr<damatrix> lhs, 
			const std::shared_ptr<damatrix> rhs, daOp oper);						// Perform operation
		// Need special function for power, as it involves an unsigned int
		static std::shared_ptr<damatrix> pow(const std::shared_ptr<damatrix> &base, 
			unsigned int power);
		virtual ~damatrix();
	private:
		void __init();
	public:
		virtual std::shared_ptr<const matrixop> eval(const mapid &valmap) const;			// Evaluate the damatrix to a matrixop
		inline bool needsRot() const {return _needsRot;}
	protected:
		std::shared_ptr<damatrix> _rootA, _rootB;
		unsigned int _pow;
		daOp _parentOp;
		bool _eval_cache_enabled;
		bool _needsRot;
		// Using boost for now, as it's faster on MSVC 2010
		mutable std::unordered_map<mapid, std::shared_ptr<const matrixop>, boost::hash<rtmath::mapid> > _eval_cache;
		//mutable std::map<mapid, std::shared_ptr<matrixop>, mmapcomp > _eval_cache;
		//mutable std::map<HASH_t, std::shared_ptr<matrixop>, hashcomp > _eval_cache;
		//mutable std::unordered_map<mapid, std::shared_ptr<matrixop> > _eval_cache;
	public: // Static Functions
		// Perform the desired operation involving two damatrices
		static std::shared_ptr<damatrix> op(const std::shared_ptr<damatrix> lhs, 
			const std::shared_ptr<damatrix> rhs, daOp oper);
	};



}; // end namespace rtmath
