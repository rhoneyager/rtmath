#include "Stdafx-voronoi.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/points.h"
#include "../rtmath/ddscat/shapefile.h"
#include "shapestats_private.h"

#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/logging_base.h>
#include <boost/log/sources/global_logger_storage.hpp>


namespace {
	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_convolute,
		boost::log::sources::severity_channel_logger_mt< >,
		(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "convolute"));
}

namespace rtmath {
	namespace ddscat
	{
		namespace points {

			shape_t convolute_B(shape_t s, size_t kernelrad)
			{
				using rtmath::ddscat::shapefile::shapefile;
				boost::shared_ptr<shapefile> res = shapefile::generate();

				Eigen::Array3i maxs = s->maxs.cast<int>(),
					mins = s->mins.cast<int>();
				mins = mins - kernelrad;
				maxs = maxs + kernelrad;
				auto spans = maxs - mins;
				auto rs = spans + 1;
				auto getIndex = [&](const Eigen::Array3f &crds) -> size_t {
					size_t index = 0;
					Eigen::Array3i s = (crds - mins.cast<float>()).cast<int>();
					index = (s(0) * (rs(1) * rs(2))) + (s(1) * rs(2)) + s(2);
					return index;
				};
				auto getCrds = [&](size_t index) -> Eigen::Array3i {
					Eigen::Array3i res;
					res(0) = index / (rs(1)*rs(2));
					index -= res(0)*rs(1)*rs(2);
					res(1) = index / rs(2);
					index -= res(1)*rs(2);
					res(2) = index;
					return res;
				};
				const size_t sz = rs.prod();
				size_t num = 0;
				Eigen::Array3d run; run.setZero();
				auto ptsearch = ::rtmath::ddscat::points::points::generate(
					s->latticePts);
				::rtmath::ddscat::shapefile::decimationFunction df;
				df = std::bind(
					::rtmath::ddscat::points::points::convolutionNeighborsRadius,
					std::placeholders::_1,std::placeholders::_2,
					(double)kernelrad,ptsearch);
				std::vector<::rtmath::ddscat::shapefile::convolutionCellInfo> vals(sz);
				for (size_t i=0; i<sz; ++i) {
					auto &v = vals.at(i);
					v.initDiel = 1;
					v.numTotal = 1;
					v.s.setConstant(1);
					// Needs to be added to mins to get proper offset.
					auto c = getCrds(i);
					v.crd = c.cast<float>() + s->mins.cast<float>();
					v.numFilled = df(v,s);
					if (v.numFilled) {
						num++; run = run + v.crd.cast<double>();
					}
				}

				res->a1 = s->a1;
				res->a2 = s->a2;
				res->a3 = s->a3;
				res->d = s->d;
				res->desc = s->desc;
				res->Dielectrics = s->Dielectrics;
				res->filename = s->filename;
				res->xd = s->xd;

				run = run / (double) num;
				Eigen::Array3f runf = run.cast<float>();
				Eigen::Matrix<float, Eigen::Dynamic,3> runft;
				runft.resize(1,3);
				runft = runf.matrix().transpose() * (-1);
				res->resize(num);

				size_t dielMax = 0;
				// Set the decimated values
				size_t point = 0;
				for (size_t i = 0; i < vals.size(); ++i)
				{
					auto &v = vals.at(i);
					if (v.numFilled == 0) continue;
					auto t = getCrds(i);
					auto crdsm = res->latticePts.block<1, 3>(point, 0);
					auto crdsi = res->latticePtsRi.block<1, 3>(point, 0);
					auto crdss = res->latticePtsStd.block<1, 3>(point, 0);
					auto crdsn = res->latticePtsNorm.block<1, 3>(point, 0);
					crdsm = t.cast<float>() + mins.cast<float>();
					crdsi.setConstant((float)v.numFilled);

					crdss = crdsm + runft;
					crdsn = crdsm + runft;
					if (dielMax < v.numFilled) dielMax = v.numFilled;
					point++;
				}

				// Set the dielectrics
				res->Dielectrics.clear();
				for (size_t i = 1; i <= dielMax; ++i)
					res->Dielectrics.insert(i);

				res->recalcStats();
				// Rescale x0 to point to the new center
				res->x0 = res->means;

				return res;
			}

		}
	}
}

