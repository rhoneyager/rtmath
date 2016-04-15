/// \brief Provides flann
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <flann/flann.h>
#include <flann/flann.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/points.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-flann.h"

namespace rtmath {
	namespace plugins {
		namespace flann {
			class flannPoints :
				public ::rtmath::ddscat::points::points {
			private:
				bool constructed;
				::rtmath::ddscat::points::backend_type src;
				Eigen::Matrix<float,Eigen::Dynamic, 3, Eigen::RowMajor> pts;
				boost::shared_ptr< ::flann::Matrix<float> > fd;
				boost::shared_ptr< ::flann::Index<::flann::L2<float> > > index;
			public:
				virtual size_t neighborSearchRadius(
					float radiussq,
					const Eigen::Array3f &pt,
					::rtmath::ddscat::points::backend_index_type &outpoints,
					::rtmath::ddscat::points::backend_scalar_type &outdists) const {

					Eigen::Matrix<float,1, 3, Eigen::RowMajor> q;
					q(0,0) = pt(0); q(0,1) = pt(1); q(0,2) = pt(2);
					::flann::Matrix<float> query(q.data(), 1, 3);
					Eigen::Matrix<size_t,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> indices;
					indices.resize(pts.rows(), 1);
					Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists;
					dists.resize(pts.rows(), 1);

					// FLANN truncates the neighbor search based on the
					// number of COLUMNS of these matrices. They should really
					// have looked at the total size instead.
					::flann::Matrix<size_t> qindices(indices.data(), 1, pts.rows());
					::flann::Matrix<float> qdists(dists.data(), 1, pts.rows());

					size_t count = index->radiusSearch(query, qindices, qdists, radiussq, ::flann::SearchParams(128));
					indices.conservativeResize((int) count, 1);
					dists.conservativeResize((int) count, 1);
					outpoints = indices;
					outdists = dists;
					return count;
				}
				virtual size_t nearestNeighbors(
					size_t N,
					const Eigen::Array3f &pt,
					::rtmath::ddscat::points::backend_index_type &outpoints,
					::rtmath::ddscat::points::backend_scalar_type &outdists) const {

					Eigen::Matrix<float,1, 3, Eigen::RowMajor> q;
					q(0,0) = pt(0); q(0,1) = pt(1); q(0,2) = pt(2);
					::flann::Matrix<float> query(q.data(), 1, 3);
					Eigen::Matrix<size_t,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> indices;
					indices.resize(N, 1);
					Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists;
					dists.resize(N, 1);

					::flann::Matrix<size_t> qindices(indices.data(), 1, N);
					::flann::Matrix<float> qdists(dists.data(), 1, N);

					size_t count = index->knnSearch(query, qindices, qdists, N, ::flann::SearchParams(128));
					outpoints = indices;
					outdists = dists;

					return count;
				}
				virtual void constructTree() {
					if (constructed) return;
					// Expected in row-major format. Eigen is in column-major.
					pts = src;
					// Construct the wrappers and build the index table.
					fd = boost::shared_ptr<::flann::Matrix<float> >(
						new ::flann::Matrix<float> (pts.data(),
						(size_t) pts.rows(), (size_t) pts.cols()));
					index = boost::shared_ptr< ::flann::Index<::flann::L2<float> > >(
						new ::flann::Index<::flann::L2<float> > (*(fd.get()),
							::flann::KDTreeIndexParams(4)));
					index->buildIndex();
					constructed = true;
				}

				flannPoints() : constructed(false) {}
				virtual ~flannPoints() {
				}

				static boost::shared_ptr<points> generate(
					::rtmath::ddscat::points::backend_type src) {
					boost::shared_ptr<flannPoints> res(new flannPoints);
					res->src = src;
					res->constructTree();
					return res;
					}
				};
		}
	}
}


D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::flann;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-flann",
		"Provides K-d tree searches",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::ddscat::points::points_provider<::rtmath::ddscat::points::points> reg_flann;
	reg_flann.name = PLUGINID;
	reg_flann.generator = ::rtmath::plugins::flann::flannPoints::generate;

	doRegisterHook<rtmath::ddscat::points::points,
		::rtmath::ddscat::points::points_provider_registry,
		::rtmath::ddscat::points::points_provider<::rtmath::ddscat::points::points>
			>(reg_flann);
	return SUCCESS;
}
