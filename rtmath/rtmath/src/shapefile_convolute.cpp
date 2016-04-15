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

			/** convolute_A first constructs an empty result matrix.
			 * It performs 'stamping' using each initial ice lattice site.
			 * The result matrix is the number of lattice sites within range.
			 **/
			shape_t convolute_A(shape_t s, size_t kernelrad) {
				using rtmath::ddscat::shapefile::shapefile;


				// maxs and mins are for the OUTPUT matrix.
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
				// cmat is the 3d block convolution matrix. The coordinates are
				// determined using getIndex and getCrds.
				Eigen::Array<int, Eigen::Dynamic, 1> cmat;
				cmat.resize((int) sz, 1);
				cmat.setZero();

				// Also need the spherical convolution matrix
				auto sVol = sphereVol::generate(kernelrad);
				// sData is an array of { x, y, z, filled }
				// Sphere is centerd at the origin!
				auto sData = sVol->getData();
				int sSz = sData->rows();
				int sNumPtsF = sVol->pointsInSphere();
				// Converts the points in sVol to a list of indices that must
				// be incremented.
				auto translate = [&](const Eigen::Array3i pt) -> Eigen::ArrayXi {
					Eigen::ArrayXi res;
					res.resize(sNumPtsF,1);
					int idx = 0;
					for (int i=0; i < sSz; ++i) {
						auto blk = sData->block<1,4>(i,0);
						auto blkc = sData->block<1,3>(i,0);
						if (blk(0,3) > 0) {
							// This is a points that must be translated
							Eigen::Array3i t = pt + blkc.transpose();
							size_t outindex = getIndex(t.cast<float>());
							res(idx) = (int) outindex;
							idx++;
						}
					}
					return res;
				};
				// To get the convolution, iterate over all points in the
				// source shape. Pass each point to the translation function. It
				// outputs a list of indices in the final matrix that get incremented.
				// So, I can run this in parallel over all input points and perform
				// the summation at the end.
				Eigen::ArrayXi totlist;
				totlist.resize((int) s->numPoints * sNumPtsF, 1);
				totlist.setZero();
				for (int i=0; i < s->numPoints; ++i) {
					Eigen::Array3i pt(
						(int) s->latticePts(i,0),
						(int) s->latticePts(i,1),
						(int) s->latticePts(i,2));
					auto resTrans = translate(pt);
					totlist.block(i * sNumPtsF,0,sNumPtsF,1) = resTrans;
				}
				for (int i=0; i < totlist.rows(); ++i) {
					int idx = totlist(i);
					cmat(idx) = cmat(idx) + 1;
				}
				// Put cmat back into a shapefile.
				boost::shared_ptr<shapefile> res = shapefile::generate();
				res->a1 = s->a1;
				res->a2 = s->a2;
				res->a3 = s->a3;
				res->d = s->d;
				res->desc = s->desc;
				res->Dielectrics = s->Dielectrics;
				res->filename = s->filename;
				res->xd = s->xd;

				int finalnumpts = cmat.count();
				res->resize(finalnumpts);

				size_t dielMax = 0;
				// Set the decimated values
				size_t point = 0;
				for (int i = 0; i < cmat.rows(); ++i)
				{
					if (cmat(i) == 0) continue;
					auto t = getCrds(i);
					auto crdsm = res->latticePts.block<1, 3>(point, 0);
					auto crdsi = res->latticePtsRi.block<1, 3>(point, 0);
					crdsm = t.cast<float>() + mins.cast<float>();
					crdsi.setConstant(cmat(i));

					if (dielMax < cmat(i)) dielMax = cmat(i);
					point++;
				}
				res->recalcStats();
				res->latticePtsStd.array().block(0,0,finalnumpts,1)
					= res->latticePts.array().block(0,0,finalnumpts,1) - res->means(0);
				res->latticePtsStd.array().block(0,1,finalnumpts,1)
					= res->latticePts.array().block(0,1,finalnumpts,1) - res->means(1);
				res->latticePtsStd.array().block(0,2,finalnumpts,1)
					= res->latticePts.array().block(0,2,finalnumpts,1) - res->means(2);
				res->latticePtsNorm = res->latticePtsStd;

				// Set the dielectrics
				res->Dielectrics.clear();
				for (size_t i = 1; i <= dielMax; ++i)
					res->Dielectrics.insert(i);

				// Rescale x0 to point to the new center
				res->x0 = res->means;

				return res;
			}

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
					// Was off by kernelrad+1 rel to convolution_A, which was correct.
					v.crd = c.cast<float>() + s->mins.cast<float>() - kernelrad - 1;
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

