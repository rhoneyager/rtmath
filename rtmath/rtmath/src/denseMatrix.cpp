#include "../rtmath/Stdafx.h"

#include "../rtmath/denseMatrix.h"

namespace rtmath {

	denseMatrix::denseMatrix(size_t sx, size_t sy, size_t sz) : wsz(32)
	{
		resize(sx,sy,sz);
	}

	void denseMatrix::resize(size_t sx, size_t sy, size_t sz)
	{
		size_t ax = sx, ay = sy;
		size_t az = sz / wsz, rz = sz % wsz;
		if (rz) az++;

		dataSize = ax * ay * az;
		data = boost::shared_array<boost::uint32_t>(new boost::uint32_t[dataSize]);
		std::fill_n(data.get(), dataSize, 0);
		sX = ax;
		sY = ay;
		sZ = az;
		numOn = 0;
	}

	bool denseMatrix::get(size_t x, size_t y, size_t z)
	{
		boost::uint32_t *wd = nullptr, test = 0;
		size_t offset;
		locate(x,y,z,&wd,offset);
		test = *wd & bit(offset);
		return (test) ? true : false;
	}

	void denseMatrix::set(size_t x, size_t y, size_t z, bool val)
	{
		boost::uint32_t *wd = nullptr;
		size_t offset;
		locate(x,y,z,&wd,offset);

		if (get(x,y,z) != val)
			(val) ? numOn++ : numOn--;

		if (val)
			*wd = *wd | bit(offset);
		else
			*wd = *wd & ~bit(offset);
	}

	void denseMatrix::locate(size_t x, size_t y, size_t z, boost::uint32_t **wd, size_t &offset)
	{
		size_t az = z / wsz;
		size_t index = (x * sY * sZ) + (y * sZ) + az;
		*wd = data.get()+index;
		offset = z % wsz;
	}

	boost::uint32_t denseMatrix::bit(size_t offset)
	{
		return 1 << offset;
	}
}

