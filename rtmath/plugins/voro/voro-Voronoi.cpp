#pragma warning( disable : 4996 ) // crt fopen
#pragma warning( disable : 4244 ) // convertion from int64 to int
#define _CRT_SECURE_NO_WARNINGS
#include "../../rtmath/rtmath/defs.h"
#include <cstdio>
#include <functional>
#include <scoped_allocator>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_heap_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/flat_map.hpp>
#include <boost/interprocess/containers/flat_set.hpp>
#include <boost/interprocess/containers/set.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/unordered_set.hpp>
#include <voro++.hh>
#include <Ryan_Debug/debug.h>
//#include <Ryan_Serialization/serialization.h>
#include <string>
#include <Ryan_Debug/hash.h>
#include "../../rtmath/rtmath/depGraph.h"
#include "plugin-voro.h"
#include "voro-CachedVoronoi.h"
#include "voro-Voronoi.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/Voronoi/CachedVoronoi.h"
#include <Ryan_Debug/Serialization.h>
#include "../../rtmath/rtmath/error/debug.h"
#include <Ryan_Debug/error.h>

namespace {
	// Golden ratio constants
	const double Phi = 0.5*(1 + sqrt(5.0));
	const double phi = 0.5*(1 - sqrt(5.0));

	class wall_initial_shape : public voro::wall {
	public:
		wall_initial_shape() {
			const double w = 2;
			v.init(-w, w, -w, w, -w, w);
			// Create a dodecahedron
			//v.plane(0,Phi,1);v.plane(0,-Phi,1);v.plane(0,Phi,-1);
			//v.plane(0,-Phi,-1);v.plane(1,0,Phi);v.plane(-1,0,Phi);
			//v.plane(1,0,-Phi);v.plane(-1,0,-Phi);v.plane(Phi,1,0);
			//v.plane(-Phi,1,0);v.plane(Phi,-1,0);v.plane(-Phi,-1,0);
		};
		bool point_inside(double x, double y, double z) { return true; }
		bool cut_cell(voro::voronoicell &c, double x, double y, double z) {

			// Set the cell to be equal to the dodecahedron
			c = v;
			return true;
		}
		bool cut_cell(voro::voronoicell_neighbor &c, double x, double y, double z) {

			// Set the cell to be equal to the dodecahedron
			c = v;
			return true;
		}
	private:
		voro::voronoicell v;
	};
	wall_initial_shape wis;

	/// \brief Draws a polygon in POV-ray format
	/// \note Taken from voro++ ploygons example, with c-style io translated to c++-style
	/// \todo Change to a template to handle different vector allocator types
	void drawPOV_polygon(FILE *fp, std::vector<int> &f_vert, std::vector<double> &v, int j)
	{
		//std::vector<std::string> s(600);
		static char s[30][128];
		int k, l, n = f_vert[j];

		// Create POV-Ray vector strings for each of the vertices
		for (k = 0; k < n; k++) {
			l = 3 * f_vert[j + k + 1];
			sprintf(s[k], "<%g,%g,%g>", v[l], v[l + 1], v[l + 2]);
		}

		// Draw the interior of the polygon
		fputs("union{\n", fp);
		for (k = 2; k < n; k++)
			fprintf(fp, "\ttriangle{%s,%s,%s}\n", s[0], s[k - 1], s[k]);
		fputs("\ttexture{t1}\n}\n", fp);

		// Draw the outline of the polygon
		fputs("union{\n", fp);
		for (k = 0; k < n; k++) {
			l = (k + 1) % n;
			fprintf(fp, "\tcylinder{%s,%s,r}\n\tsphere{%s,r}\n",
				s[k], s[l], s[l]);
		}
		fputs("\ttexture{t2}\n}\n", fp);
	}
}


namespace rtmath
{
	namespace plugins
	{
		namespace voro
		{
			using namespace ::rtmath::Voronoi;

			VoroVoronoiDiagram::VoroVoronoiDiagram()
			{
				pluginId = std::string(PLUGINID);
			}
			VoroVoronoiDiagram::~VoroVoronoiDiagram()
			{
			}

			double VoroVoronoiDiagram::surfaceArea() const
			{
				if (!cache.count("precalced")) return 0;
				return cache.at("precalced")->surfaceArea();
			}

			double VoroVoronoiDiagram::volume() const
			{
				if (!cache.count("precalced")) return 0;
				return cache.at("precalced")->volume();
			}

			void VoroVoronoiDiagram::calcFv(size_t threshold, size_t &innerPointsTotal, size_t &innerPointsFilled) const
			{
				auto depthSfc = calcSurfaceDepth();
				auto cellmap = getCellMap();

				//boost::shared_ptr<Eigen::MatrixXi > resPts(new Eigen::MatrixXi());
				//resPts->resize(shp->numPoints, 3);
				size_t outerCount = 0; // Keeps track of the number of points on the 'outside'.

				// Keep track of the inner volume fraction
				innerPointsTotal = 0;
				innerPointsFilled = 0;

				// Also track all potential dipole sites
				//std::vector<Eigen::Matrix3i> potentialInnerPoints;
				//potentialInnerPoints.reserve(cellmap->rows());

				// First, iterate over all of the entries in depthSfc to extract the constant surface points.
				for (int i = 0; i < depthSfc->rows(); ++i)
				{
					// First three arguments are the coordinate. Fourth is the surface depth.
					auto crds = depthSfc->block<1, 3>(i, 0);
					int depth = (int)(*depthSfc)(i, 3);
					if (depth < threshold)
					{
						//resPts->block<1, 3>(outerCount, 0) = crds.cast<int>();
						outerCount++;
					}
					else innerPointsFilled++;
				}

				// Iterate over all entries in the cell map, while consulting the surface depth field.
				for (int i = 0; i < cellmap->rows(); ++i)
				{
					// i is the probe point id
					// The first three columns are the probe point coordinates
					// The fourth column is the the voronoi point id (row)
					auto crds = cellmap->block<1, 3>(i, 0);
					int voroid = (*cellmap)(i, 3);
					int depth = (int)(*depthSfc)(voroid, 3);
					if (depth >= threshold)
					{
						innerPointsTotal++;
						//Eigen::Matrix3i p;
						//p.block<1, 3>(0, 0) = crds.block<1, 3>(0, 0);
						//potentialInnerPoints.push_back(std::move(p));
					}
				}

				/*
				cout << "There are " << shp->numPoints << " points total.\n"
					<< "The threshold is " << threshold << ".\n"
					<< "The surface has " << outerCount << " points.\n"
					<< "The interior has " << innerPointsFilled << " filled points, "
					"and " << innerPointsTotal << " total potential sites.\n"
					<< "The inner fraction is " << 100.f * (float)innerPointsFilled
					/ (float)innerPointsTotal << " percent." << std::endl;
				*/
			}

			void VoroVoronoiDiagram::getBounds(Eigen::Array3f &mins,
				Eigen::Array3f &maxs, Eigen::Array3f &span) const
			{
				mins = this->mins;
				maxs = this->maxs;
				span = maxs - mins + 1;
			}

			void VoroVoronoiDiagram::getResultsTable(std::map<std::string, VoronoiDiagram::matrixType> &res) const
			{
				res = results;
			}

			/// \note This regenerates the REGULAR diagram, with surface fitting
			void VoroVoronoiDiagram::regenerateVoronoi() const
			{
				if (vc) return;
				// Set up the number of blocks that the container is divided into
				int n_x = 50, n_y = 50, n_z = 50, init_grid = 120;
				using namespace ::voro;
				auto preVc = boost::shared_ptr<::voro::pre_container>(new ::voro::pre_container(
					mins(0) - 1, maxs(0) + 1, mins(1) - 1, maxs(1) + 1, mins(2) - 1, maxs(2) + 1, false, false, false));


				// Add particles into the container
				for (size_t i = 0; i < (size_t)src->rows(); ++i)
				{
					auto pt = src->block<1, 3>(i, 0);
					preVc->put((int)i, pt(0), pt(1), pt(2));
				}
				preVc->guess_optimal(n_x, n_y, n_z);

				vc = boost::shared_ptr<::voro::container>(new ::voro::container(
					mins(0) - 1, maxs(0) + 1, mins(1) - 1, maxs(1) + 1, mins(2) - 1, maxs(2) + 1,
					n_x, n_y, n_z, false, false, false, init_grid));
				//wall_initial_shape wis;
				vc->add_wall(wis);
				preVc->setup(*vc);

			}

			/** This function iterates over the entire diagram, generating the data for all
			* of the cells, with regards to cell ids, voronoi centers and neighbors.
			* As needed, will add information about cell vertices and faces.
			* This is needed to prevent repetitive looping over the Voronoi structure.
			**/
			void VoroVoronoiDiagram::regenerateFull() const
			{
				if (cache.count("precalced")) return;
				if (!vc) regenerateVoronoi();
				cache["precalced"] = boost::shared_ptr<VoroCachedVoronoi>(new VoroCachedVoronoi((size_t)src->rows(), vc, mins, maxs));
			}

			boost::shared_ptr<const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> > VoroVoronoiDiagram::getCellMap() const
			{
				regenerateFull();  return cache.at("precalced")->getCellMap();
			}

			size_t VoroVoronoiDiagram::numPoints() const
			{
				return (size_t)src->rows();
			}

			Eigen::Array3i VoroVoronoiDiagram::getSpan() const
			{
				Eigen::Array3f span = maxs - mins + 1;
				Eigen::Array3i si = span.cast<int>();
				return si;
			}

			::rtmath::Voronoi::VoronoiDiagram::matrixType VoroVoronoiDiagram::calcSurfaceDepth() const
			{
				if (results.count("SurfaceDepth"))
				{
					return results.at("SurfaceDepth");
				}
				regenerateFull();

				using namespace voro;
				boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > out
					(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(*src));
				out->conservativeResize(src->rows(), 4);
				out->col(3).setZero();

				boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > outVectors
					(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(*src));
				outVectors->conservativeResize(src->rows(), 6);
				outVectors->col(3).setZero();
				outVectors->col(4).setZero();
				outVectors->col(5).setZero();

				boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > outNNeighs
					(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(*src));
				outNNeighs->conservativeResize(src->rows(), 4);
				outNNeighs->col(3).setZero();

				boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > outOrder
					(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(*src));
				outOrder->conservativeResize(src->rows(), 4);
				outOrder->col(3).setZero();

				// Using depGraph and the initial Candidate Convex Hull points

				// Construct the dependency graph
				using namespace rtmath::graphs;
				/// \todo Support exporting the vertices (with connection information) using 
				/// serialization and graphical (i.e. silo) output.
				std::vector<vertex> vertices((size_t)src->rows());

				using namespace boost::interprocess;

				//mapped_region region(anonymous_shared_memory(1024*1024*128));
				/*
				managed_heap_memory &m = cache["precalced"]->m;
				typedef allocator<std::pair<const vertex*, size_t>, managed_heap_memory::segment_manager>
				PairAllocator;
				const PairAllocator pairAllocator(m.get_segment_manager());
				typedef boost::interprocess::flat_map<vertex*, size_t, std::less<const vertex*>,
				PairAllocator> vIdMap;

				vIdMap *vertexIdMap = m.construct<vIdMap>("SurfaceDepth_vertexIdMap")
				(std::less<const vertex*>(), pairAllocator);
				*/
				//std::unordered_map<vertex*, size_t, std::hash<vertex*>, std::equal_to<vertex*>,
				//	boost::pool_allocator<std::pair<const vertex*, size_t> > > vertexIdMap;
				std::unordered_map<vertex*, size_t, std::hash<vertex*>, std::equal_to<vertex*> > vmap;
				auto vertexIdMap = &vmap;

				vertexIdMap->reserve(src->rows());
				for (size_t i = 0; i < (size_t)src->rows(); ++i)
				{
					//vertices[i] = boost::shared_ptr<vertex>(new vertex(true) );
					vertices[i].setOR(true);
					(*vertexIdMap)[&vertices[i]] = i;
				}

				// Iterate over the precalced entries and insert only point that touch a boundary
				//for (const auto &cell : *(precalced->getCells()))
				for (size_t row = 0; row < (size_t)cache["precalced"]->tblDoubles->rows(); ++row)
				{
					auto od = cache["precalced"]->tblDoubles->block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(row, 0);
					auto oi = cache["precalced"]->tblInts->block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(row, 0);

					const int &id = oi(CachedVoronoi::ID);
					if (id % 1000 == 0)
						std::cerr << id << "\n";

					//for (auto &i : cell.neigh)
					for (size_t col = 0; col < CachedVoronoi_MaxNeighbors_VAL; ++col)
					{
						size_t i = static_cast<size_t>((*cache["precalced"]->tblCellNeighs)(row, col));

						if (i <= 0) continue;
						auto distsq = [&](size_t i, size_t j) -> float
						{
							float res = (out->block(i, 0, 1, 3) - out->block(j, 0, 1, 3)).norm();
							return res;
						};

						if (distsq(i, id) < 2.2f)
						{
							if (vertices.size() > id)
							{
								vertices[id].addSlot(&vertices[i]);
								(*outNNeighs)(id, 3)++;
								//std::cerr << "\tAttaching " << id+1 << " (" << &vertices[id]
								//	<< ") to " << i+1 << " (" << &vertices[i] << ").\n";
							}
							else {
								// Something bad is happening
								std::cerr << "Error\n";
							}
						}
					}
				}

				// Construct the set of vertex pointers from the vertex vector
				typedef allocator<vertex*, managed_heap_memory::segment_manager> VertexAllocator;
				//typedef boost::interprocess::set<vertex*, std::less<vertex*>, VertexAllocator> bSetVertex;
				//typedef boost::interprocess::flat_set<vertex*, std::less<const vertex*>, VertexAllocator> bSetVertex;
				typedef boost::unordered_set < vertex*, boost::hash<vertex*>,
					std::equal_to < vertex* >
					//, VertexAllocator
				> bSetVertex;
				//VertexAllocator vAllocator (m.get_segment_manager());

				bSetVertex setVertices; //(std::less<vertex*>(), vAllocator);

				//bSetVertex* setVertices = m.construct<bSetVertex>("setVertices")
				//(boost::hash<vertex*>(), std::equal_to<vertex*>(),
				//(vAllocator);
				//typedef std::unordered_set < vertex*, std::hash<vertex*>, std::equal_to<vertex*>,
				//	boost::pool_allocator<vertex*> > bSetVertex;
				//bSetVertex setVertices;
				//setVertices.reserve((size_t)src->rows());
				//rtmath::graphs::setVertex setVertices; //(vertices.begin(), vertices.end());
				for (auto &v : vertices)
					setVertices.insert(&v);
				orderedVertex order;
				bSetVertex remaining;
				bSetVertex ignored;
				bSetVertex provided;

				//auto initFilledPoints = calcCandidateConvexHullPoints();
				/*
				Eigen::MatrixXf xinitFilledPoints(4, 4);
				Eigen::MatrixXf *initFilledPoints = &xinitFilledPoints;
				{
				auto cell = cache["precalced"]->getCells()->begin();
				for (size_t k = 0; k < 4; k++)
				{
				const int &id = cell->id;
				xinitFilledPoints.block<1, 3>(k, 0) = out->block(id, 0, 1, 3);
				xinitFilledPoints(k, 3) = (float)id;
				cerr << "Starting point is: " << id + 1 << " at coords "
				<< (*out)(id, 0) << ", " << (*out)(id, 1) << ", " << (*out)(id, 2) << "\n";
				cell++;
				}
				}
				*/
				auto initFilledPoints = calcPointsSAfracExternal();
				//cerr << *initFilledPoints << endl;
				for (size_t i = 0; i < (size_t)initFilledPoints->rows(); ++i)
				{
					//size_t index = (size_t)(*initFilledPoints)(i, 3);
					float frac = (*initFilledPoints)(i, 3);
					if (frac < 0.01) continue;
					provided.insert(&vertices[i]);
					//cerr << "Matching on " << i << ": " << &vertices[i] << endl;
				}

				generateGraph<bSetVertex, orderedVertex>::generate(
					setVertices, provided, order, remaining, ignored);

				//graph g(setVertices);
				//g.generate(provided, order, remaining, ignored);

				// Provided all have rank zero. order provides depth from the candidate surface cells.
				// Match the ordered vertices with their row
				size_t k = 0;
				for (auto &it : order)
				{
					const size_t &rank = std::get<1>(it); // it.second;
					auto IT = std::get<0>(it); // .first;//.lock();
					auto parent = std::get<2>(it);
					size_t id = vertexIdMap->at(IT);
					(*out)(id, 3) = (float)rank;
					if (parent)
					{
						size_t parentID = vertexIdMap->at(parent);
						outVectors->block<1, 3>(id, 3) =
							outVectors->block<1, 3>(parentID, 0) - outVectors->block<1, 3>(id, 0);
					}
					else {
						outVectors->block<1, 3>(id, 3).setZero();
					}
					(*outOrder)(id, 3) = k;
					k++;
					//if (rank)
					/*
					{
					cerr << "Point: " << id+1 << " at " << IT << " with rank " << rank << " and coords "
					<< (*outVectors)(id, 0) << ", " << (*outVectors)(id, 1) << ", " << (*outVectors)(id, 2) << "\n";
					if (parent)
					{
					size_t parentID = vertexIdMap->at(parent);
					cerr << "\tFilled via point " << parentID+1 << ": "
					<< (*outVectors)(parentID, 0) << ", " << (*outVectors)(parentID, 1) << ", " << (*outVectors)(parentID, 2)
					<< " (" << parent << ")\n";
					}
					}
					*/
				}

				std::cerr << std::endl;

				// Clean up vertex graph
				//m.destroy_ptr(vertexIdMap);

				results["SurfaceDepth"] = out;
				results["SurfaceDepthVectors"] = outVectors;
				results["SurfaceDepthNumNeighbors"] = outNNeighs;
				results["SurfaceDepthFillingOrder"] = outOrder;
				return out;
			}

			::rtmath::Voronoi::VoronoiDiagram::matrixType VoroVoronoiDiagram::calcSurfaceDepthVectors() const
			{
				// calcSurfaceDepth takes care of everything. This function just returns its second matrix.
				calcSurfaceDepth();
				return results.at("SurfaceDepthVectors");
			}

			::rtmath::Voronoi::VoronoiDiagram::matrixType VoroVoronoiDiagram::calcSurfaceNumNeighs() const
			{
				// calcSurfaceDepth takes care of everything. This function just returns its second matrix.
				calcSurfaceDepth();
				return results.at("SurfaceDepthNumNeighbors");
			}

			::rtmath::Voronoi::VoronoiDiagram::matrixType VoroVoronoiDiagram::calcSurfaceFillingOrder() const
			{
				// calcSurfaceDepth takes care of everything. This function just returns its second matrix.
				calcSurfaceDepth();
				return results.at("SurfaceDepthFillingOrder");
			}

			::rtmath::Voronoi::VoronoiDiagram::matrixType
				VoroVoronoiDiagram::calcPointsSAfracExternal() const
			{
				if (results.count("SAfracExternal"))
				{
					return results.at("SAfracExternal");
				}
				// Computational cost should be around the same with / without first preselecting 
				// surface points from the more complete diagram. I usually need the full diagram, 
				// though, and it is more expensive to calculate in the other order.
				regenerateFull();

				using namespace voro;
				boost::shared_ptr< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > out(
					new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(src->rows(), 4));
				out->setZero();

				// Check each particle to see if on the container surface
				size_t numSurfacePoints = 0;
				size_t numPointCells = 0;

				// Using the precalced loop here
				for (size_t i = 0; i < (size_t)cache["precalced"]->tblDoubles->rows(); ++i)
					//for (const auto &cell : *(precalced->getCells()))
				{
					auto od = cache["precalced"]->tblDoubles->block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(i, 0);
					auto oi = cache["precalced"]->tblInts->block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(i, 0);

					//if (cell.isSurface())
					double extVfrac = od(CachedVoronoi::SA_EXT) / od(CachedVoronoi::SA_FULL); // Exterior volume fraction test
					if (!od(CachedVoronoi::SA_FULL))
					{
						//cerr << "Point cell at " << cell.pos(0) << ", " << cell.pos(1) << ", " << cell.pos(2) << endl;
						//extVfrac = 0; // Oddly occurs in some of the cells
						numPointCells++;
						continue;
					}

				{
					(*out)(numSurfacePoints, 0) = (float)od(CachedVoronoi::POS_X);
					(*out)(numSurfacePoints, 1) = (float)od(CachedVoronoi::POS_Y);
					(*out)(numSurfacePoints, 2) = (float)od(CachedVoronoi::POS_Z);
					(*out)(numSurfacePoints, 3) = (float)extVfrac; // cell.id; // Initial point id
					numSurfacePoints++;
				}
				}

				if (numPointCells)
					std::cerr << "Number of point cells: " << numPointCells << std::endl;


				out->conservativeResize(numSurfacePoints, 4);

				results["SAfracExternal"] = out;
				return out;
			}

			/// This uses a separate voronoi container that is 'unshrunk' to get the prospective hull points.
			::rtmath::Voronoi::VoronoiDiagram::matrixType
				VoroVoronoiDiagram::calcCandidateConvexHullPoints() const
			{
				if (results.count("CandidateConvexHullPoints"))
				{
					return results.at("CandidateConvexHullPoints");
				}
				// Computational cost should be around the same with / without first preselecting 
				// surface points from the more complete diagram. I usually need the full diagram, 
				// though, and it is more expensive to calculate in the other order.
				regenerateFull();

				using namespace voro;
				// From the full diagram, extract the surface points only.
				// Need to regenerate the cells without a prespecified size.
				int n_x = 50, n_y = 50, n_z = 50, init_grid = 40;
				using namespace voro;
				auto preVc = boost::shared_ptr<::voro::pre_container>(new ::voro::pre_container(
					mins(0) - 1, maxs(0) + 1, mins(1) - 1, maxs(1) + 1, mins(2) - 1, maxs(2) + 1, false, false, false));

				// Iterate over the precalced entries and insert only point that touch a boundary
				size_t numCells = 0;
				for (size_t i = 0; i < (size_t)cache["precalced"]->tblDoubles->rows(); ++i)
					//for (const auto &cell : *(cache["precalced"]->getCells()))
				{
					auto od = cache["precalced"]->tblDoubles->block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(i, 0);
					auto oi = cache["precalced"]->tblInts->block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(i, 0);

					//if (cell.isSurface())
					if (od(CachedVoronoi::SA_EXT))
					{
						preVc->put((int)numCells,
							(float)od(CachedVoronoi::POS_X),
							(float)od(CachedVoronoi::POS_Y),
							(float)od(CachedVoronoi::POS_Z));
						numCells++;
					}
				}


				boost::shared_ptr<::voro::container> vcSmall(new ::voro::container(
					mins(0) - 1, maxs(0) + 1, mins(1) - 1, maxs(1) + 1, mins(2) - 1, maxs(2) + 1,
					n_x, n_y, n_z, false, false, false, init_grid));

				preVc->setup(*vcSmall);

				// It's optional to use a pool here, since this Voronoi diagram is much smaller than the other.
				cache["precalcedSmall"] = boost::shared_ptr<VoroCachedVoronoi>(new VoroCachedVoronoi(numCells, vcSmall, mins, maxs));

				using namespace voro;
				boost::shared_ptr< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > out(
					new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(src->rows(), 4));
				out->setZero();

				// Check each particle to see if on the container surface
				size_t numSurfacePoints = 0;

				// Using the precalced loop here

				for (size_t i = 0; i < (size_t)cache["precalcedSmall"]->tblDoubles->rows(); ++i)
					//for (const auto &cell : *(precalcedSmall->getCells()))
				{
					auto od = cache["precalcedSmall"]->tblDoubles->block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(i, 0);
					auto oi = cache["precalcedSmall"]->tblInts->block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(i, 0);

					if (od(CachedVoronoi::SA_EXT))
						//if (cell.isSurface())
						//double extVfrac = cell.sa_ext / cell.sa_full; // Exterior volume fraction test
					{
						(*out)(numSurfacePoints, 0) = (float)od(CachedVoronoi::POS_X);
						(*out)(numSurfacePoints, 1) = (float)od(CachedVoronoi::POS_Y);
						(*out)(numSurfacePoints, 2) = (float)od(CachedVoronoi::POS_Z);
						(*out)(numSurfacePoints, 3) = (float)oi(CachedVoronoi::ID); // Initial point id
						numSurfacePoints++;
					}
				}


				out->conservativeResize(numSurfacePoints, 4);

				// Test output to show cell boundaries for the hull
				//std::ofstream oCandidates("CandidateConvexHullPoints_extfaces.pov");
				//FILE *oCandidates=fopen("CandidateConvexHullPoints_extfaces.pov","w");
				// Some stuff for random cell highlighting (to be moved to as yet uncreated routines)
				//std::default_random_engine generator;
				//std::binomial_distribution<int> distribution(1, 0.01);
				//std::discrete_distribution<int> distribution(2,0,1,[](double d){if (d>0.1) return 1; return 2500; });
				//std::ofstream oint("CandidateConvexHullPoints_intfaces.pov");
				//drawPOV_polygon(oCandidates, f_vert, v, j);
				//vc->draw_particles_pov("CandidateConvexHullPoints_p.pov");
				//vc->draw_cells_pov("CandidateConvexHullPoints_v.pov");
				//vc->draw_particles("CandidateConvexHullPoints_p.gnu");
				//vc->draw_cells_gnuplot("CandidateConvexHullPoints_v.gnu");
				//fclose(oCandidates);

				results["CandidateConvexHullPoints"] = out;
				return out;
			}

			boost::shared_ptr<::rtmath::Voronoi::VoronoiDiagram> VoroVoronoiDiagram::generateStandard(
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &points
				)
			{
				boost::shared_ptr<VoronoiDiagram> res(new VoroVoronoiDiagram);
				res->mins = mins;
				res->maxs = maxs;
				res->src = boost::shared_ptr<const Eigen::Matrix<
					float, Eigen::Dynamic, Eigen::Dynamic> >(new
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(points));
				return res;
			}

			boost::shared_ptr<::rtmath::Voronoi::VoronoiDiagram> VoroVoronoiDiagram::generateBlank()
			{
				boost::shared_ptr<VoronoiDiagram> res(new VoroVoronoiDiagram);
				return res;
			}

			boost::shared_ptr<::rtmath::Voronoi::VoronoiDiagram> VoroVoronoiDiagram::generateUpcast(
				boost::shared_ptr<const ::rtmath::Voronoi::VoronoiDiagram> p)
			{
				boost::shared_ptr<VoroVoronoiDiagram> res(new VoroVoronoiDiagram);
				//boost::shared_ptr<::rtmath::Voronoi::VoronoiDiagram> res(new VoroVoronoiDiagram);

				// Copy all of the inner attributes from VoronoiDiagram
				res->src = p->src;
				res->results = p->results;
				res->_hash = p->_hash;
				res->mins = p->mins;
				res->maxs = p->maxs;
				res->ingest_timestamp = p->ingest_timestamp;
				res->hostname = p->hostname;
				res->ingest_username = p->ingest_username;
				res->ingest_rtmath_version = p->ingest_rtmath_version;
				res->pluginId = p->pluginId;

				// Leave res->vc unset
				//res->vc;

				// Convert the cache objects from CachedVoronoi to VoroCachedVoronoi
				for (const auto &c : p->cache)
				{
					res->cache[c.first] = boost::shared_ptr<VoroCachedVoronoi>
						(new VoroCachedVoronoi(c.second, res->vc));
				}

				return res;
			}

		}
	}

}
