#pragma once
#include <boost/shared_array.hpp>
#include <boost/cstdint.hpp>

/* denseMatrix holds a dense matrix of bools. Each bool occupies one bit of 
* storage space. The matrix is aligned on 32-byte words. It is fast. */

namespace rtmath
{

	class denseMatrix
	{
	public:
		denseMatrix(size_t sx, size_t sy, size_t sz);
		bool get(size_t x, size_t y, size_t z);
		void set(size_t x, size_t y, size_t z, bool val);
		void locate(size_t x, size_t y, size_t z, boost::uint32_t **wd, size_t &offset);

		static boost::uint32_t bit(size_t offset);

		boost::shared_array<boost::uint32_t> data;
		size_t dataSize;
		size_t sX, sY, sZ;
		size_t numOn;
		const size_t wsz;
	};

}

