#pragma once

#include <memory>
#include <map>
#include <vector>
#include <unordered_map>
#include <boost/unordered_map.hpp> // Instead of std::unordered map, for now
#include "../enums.h"
#include "../matrixop.h"
#include <limits.h>	
#include "../defs.h"
#include "../error/error.h"
#include "../coords.h"

namespace rtmath {
	
	namespace griddata
	{

		enum gridOp
		{
			NONE,
			ADD,
			MULT,
			MULTCOMPONENT,
			MULTVAL,
			INV,
			POW
		};

		class gridded
		{
		protected:
			gridded();
		public:
			gridded(const std::shared_ptr<gridded> lhs, 
				const std::shared_ptr<gridded> rhs, gridOp oper);						// Perform operation
			// Need special function for power, as it involves an unsigned int
			static std::shared_ptr<gridded> pow(const std::shared_ptr<gridded> &base, 
				unsigned int power);
			// Need special function for scalar multiplication
			static std::shared_ptr<gridded> smult(const std::shared_ptr<gridded> &base, 
				double multval);
			virtual ~gridded();
		private:
			void __init();
		public:
			// Evaluate based on start coordinates and the span
			virtual std::shared_ptr<const matrixop> eval(const coords::cyclic<double> &start, 
				const coords::cyclic<double> &span) const;
			// Get the expected matrix size
			virtual void size(std::vector<size_t> &out) const;
			inline std::vector<size_t> size() const { std::vector<size_t> res; size(res); return res; }
		protected:
			std::shared_ptr<gridded> _rootA, _rootB;
			unsigned int _pow;
			double _multval;
			gridOp _parentOp;
			bool _eval_cache_enabled;
			bool _needsRot;
			// Using boost for now, as it's faster on MSVC 2010
			//mutable std::unordered_map<coords::cyclic<double>, std::shared_ptr<const matrixop>, boost::hash<rtmath::coords::cyclic<double> > > _eval_cache;
			//mutable std::map<mapid, std::shared_ptr<matrixop>, mmapcomp > _eval_cache;
			//mutable std::map<HASH_t, std::shared_ptr<matrixop>, hashcomp > _eval_cache;
			//mutable std::unordered_map<mapid, std::shared_ptr<matrixop> > _eval_cache;
		public: // Static Functions
			// Perform the desired operation involving two damatrices
			static std::shared_ptr<gridded> op(const std::shared_ptr<gridded> lhs, 
				const std::shared_ptr<gridded> rhs, gridOp oper);
		};

	} // end namespace griddata

}; // end namespace rtmath
