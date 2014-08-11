#include "Stdafx-voronoi.h"

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
#include <Voro/voro++.hh>
#include <Ryan_Debug/debug.h>
//#include <Ryan_Serialization/serialization.h>
#include <string>
#include "../rtmath/hash.h"
#include "../rtmath/depGraph.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/Voronoi/CachedVoronoi.h"
#include "../rtmath/Serialization/Serialization.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace {
	// Golden ratio constants
	const double Phi=0.5*(1+sqrt(5.0));
	const double phi=0.5*(1-sqrt(5.0));

	class wall_initial_shape : public voro::wall {
	public:
		wall_initial_shape() {
			const double w = 2;
			v.init(-w,w,-w,w,-w,w);
			// Create a dodecahedron
			//v.plane(0,Phi,1);v.plane(0,-Phi,1);v.plane(0,Phi,-1);
			//v.plane(0,-Phi,-1);v.plane(1,0,Phi);v.plane(-1,0,Phi);
			//v.plane(1,0,-Phi);v.plane(-1,0,-Phi);v.plane(Phi,1,0);
			//v.plane(-Phi,1,0);v.plane(Phi,-1,0);v.plane(-Phi,-1,0);
		};
		bool point_inside(double x,double y,double z) {return true;}
		bool cut_cell(voro::voronoicell &c,double x,double y,double z) {

			// Set the cell to be equal to the dodecahedron
			c=v;
			return true;
		}
		bool cut_cell(voro::voronoicell_neighbor &c,double x,double y,double z) {

			// Set the cell to be equal to the dodecahedron
			c=v;
			return true;
		}
	private:
		voro::voronoicell v;
	};
	wall_initial_shape wis;

	/// \brief Draws a polygon in POV-ray format
	/// \note Taken from voro++ ploygons example, with c-style io translated to c++-style
	/// \todo Change to a template to handle different vector allocator types
	void drawPOV_polygon(FILE *fp, std::vector<int> &f_vert, std::vector<double> &v,int j)
	{
		//std::vector<std::string> s(600);
		static char s[30][128];
		int k,l,n=f_vert[j];

		// Create POV-Ray vector strings for each of the vertices
		for(k=0;k<n;k++) {
			l=3*f_vert[j+k+1];
			sprintf(s[k],"<%g,%g,%g>",v[l],v[l+1],v[l+2]);
		}

		// Draw the interior of the polygon
		fputs("union{\n",fp);
		for(k=2;k<n;k++) 
			fprintf(fp,"\ttriangle{%s,%s,%s}\n",s[0],s[k-1],s[k]);
		fputs("\ttexture{t1}\n}\n",fp);

		// Draw the outline of the polygon
		fputs("union{\n",fp);
		for(k=0;k<n;k++) {
			l=(k+1)%n;
			fprintf(fp,"\tcylinder{%s,%s,r}\n\tsphere{%s,r}\n",
				s[k],s[l],s[l]);
		}
		fputs("\ttexture{t2}\n}\n",fp);
	}
}


namespace rtmath
{
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::Voronoi::VoronoiDiagram>;
		template class usesDLLregistry<
			::rtmath::Voronoi::Voronoi_IO_output_registry,
			IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram> >;

		template struct IO_class_registry_reader
			<::rtmath::Voronoi::VoronoiDiagram>;
		template class usesDLLregistry<
			::rtmath::Voronoi::Voronoi_IO_input_registry,
			IO_class_registry_reader<::rtmath::Voronoi::VoronoiDiagram> >;
	}

	namespace Voronoi
	{
		

		VoronoiDiagram::VoronoiDiagram()
		{
			hostname = Ryan_Debug::getHostname();
			ingest_username = Ryan_Debug::getUsername();
			using namespace boost::posix_time;
			using namespace boost::gregorian;
			ptime now = second_clock::local_time();
			ingest_timestamp = to_iso_string(now);
			ingest_rtmath_version = rtmath::debug::rev();
		}

		void VoronoiDiagram::setHash(HASH_t hash) { this->_hash = hash; }
		HASH_t VoronoiDiagram::hash() const { return this->_hash; }

		double VoronoiDiagram::surfaceArea() const
		{
			if (!cache.count("precalced")) return 0;
			return cache.at("precalced")->surfaceArea();
		}

		double VoronoiDiagram::volume() const
		{
			if (!cache.count("precalced")) return 0;
			return cache.at("precalced")->volume();
		}

		void VoronoiDiagram::getBounds(Eigen::Array3f &mins, 
			Eigen::Array3f &maxs, Eigen::Array3f &span) const
		{
			mins = this->mins;
			maxs = this->maxs;
			span = maxs - mins + 1;
		}

		void VoronoiDiagram::getResultsTable(std::map<std::string, VoronoiDiagram::matrixType> &res) const
		{
			res = results;
		}

		/// \note This regenerates the REGULAR diagram, with surface fitting
		void VoronoiDiagram::regenerateVoronoi() const
		{
			if (vc) return;
			// Set up the number of blocks that the container is divided into
			int n_x=50,n_y=50,n_z=50, init_grid=120;
			using namespace voro;
			auto preVc = boost::shared_ptr<voro::pre_container>(new pre_container(
				mins(0)-1,maxs(0)+1,mins(1)-1,maxs(1)+1,mins(2)-1,maxs(2)+1,false,false,false));
			
			
			// Add particles into the container
			for (size_t i=0; i < (size_t) src->rows(); ++i)
			{
				auto pt = src->block<1, 3>(i, 0);
				preVc->put((int) i, pt(0), pt(1), pt(2));
			}
			preVc->guess_optimal(n_x,n_y,n_z);

			vc = boost::shared_ptr<voro::container>(new container(
				mins(0)-1,maxs(0)+1,mins(1)-1,maxs(1)+1,mins(2)-1,maxs(2)+1,
				n_x,n_y,n_z,false,false,false,init_grid));
			//wall_initial_shape wis;
			vc->add_wall(wis);
			preVc->setup(*vc);

		}

		/** This function iterates over the entire diagram, generating the data for all 
		* of the cells, with regards to cell ids, voronoi centers and neighbors.
		* As needed, will add information about cell vertices and faces.
		* This is needed to prevent repetitive looping over the Voronoi structure.
		**/
		void VoronoiDiagram::regenerateFull() const
		{
			if (cache.count("precalced")) return;
			if (!vc) regenerateVoronoi();
			cache["precalced"] = boost::shared_ptr<CachedVoronoi>(new CachedVoronoi((size_t)src->rows(), vc, mins, maxs));
		}

		const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* VoronoiDiagram::getCellMap() const
		{
			regenerateFull();  return cache.at("precalced")->getCellMap();
		}

		size_t VoronoiDiagram::numPoints() const
		{
			return (size_t) src->rows();
		}

		Eigen::Array3i VoronoiDiagram::getSpan() const
		{
			Eigen::Array3f span = maxs - mins + 1;
			Eigen::Array3i si = span.cast<int>();
			return si;
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceDepth() const
		{
			if (results.count("SurfaceDepth"))
			{
				return results.at("SurfaceDepth");
			}
			regenerateFull();

			using namespace voro;
			boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > out
				(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(*src) );
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
			std::vector<vertex> vertices((size_t) src->rows());

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
			for (size_t i=0; i < (size_t) src->rows(); ++i)
			{
				//vertices[i] = boost::shared_ptr<vertex>(new vertex(true) );
				vertices[i].setOR(true);
				(*vertexIdMap)[&vertices[i]] = i;
			}
			
			// Iterate over the precalced entries and insert only point that touch a boundary
			//for (const auto &cell : *(precalced->getCells()))
			for (size_t row = 0; row < (size_t)cache["precalced"]->tblDoubles.rows(); ++row)
			{
				auto od = cache["precalced"]->tblDoubles.block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(row, 0);
				auto oi = cache["precalced"]->tblInts.block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(row, 0);

				const int &id = oi(CachedVoronoi::ID);
				if (id % 1000 == 0) 
					std::cerr << id << "\n";

				//for (auto &i : cell.neigh)
				for (size_t col = 0; col < CachedVoronoi_MaxNeighbors_VAL; ++col)
				{
					size_t i = static_cast<size_t>(cache["precalced"]->tblCellNeighs(row, col));

					if (i<=0) continue;
					auto distsq = [&](size_t i, size_t j) -> float
					{
						float res = (out->block(i,0,1,3) - out->block(j,0,1,3)).norm();
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
			typedef boost::unordered_set<vertex*, boost::hash<vertex*>, 
				std::equal_to<vertex*>
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
				(*out)(id, 3) = (float) rank;
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

			cerr << endl;

			// Clean up vertex graph
			//m.destroy_ptr(vertexIdMap);

			results["SurfaceDepth"] = out;
			results["SurfaceDepthVectors"] = outVectors;
			results["SurfaceDepthNumNeighbors"] = outNNeighs;
			results["SurfaceDepthFillingOrder"] = outOrder;
			return out;
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceDepthVectors() const
		{
			// calcSurfaceDepth takes care of everything. This function just returns its second matrix.
			calcSurfaceDepth();
			return results.at("SurfaceDepthVectors");
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceNumNeighs() const
		{
			// calcSurfaceDepth takes care of everything. This function just returns its second matrix.
			calcSurfaceDepth();
			return results.at("SurfaceDepthNumNeighbors");
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceFillingOrder() const
		{
			// calcSurfaceDepth takes care of everything. This function just returns its second matrix.
			calcSurfaceDepth();
			return results.at("SurfaceDepthFillingOrder");
		}

		VoronoiDiagram::matrixType
			VoronoiDiagram::calcPointsSAfracExternal() const
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
			for (size_t i = 0; i < (size_t)cache["precalced"]->tblDoubles.rows(); ++i)
			//for (const auto &cell : *(precalced->getCells()))
			{
				auto od = cache["precalced"]->tblDoubles.block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(i, 0);
				auto oi = cache["precalced"]->tblInts.block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(i, 0);

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
					(*out)(numSurfacePoints, 3) = (float) extVfrac; // cell.id; // Initial point id
					numSurfacePoints++;
				}
			}

			if (numPointCells)
				cerr << "Number of point cells: " << numPointCells << endl;

			
			out->conservativeResize(numSurfacePoints, 4);

			results["SAfracExternal"] = out;
			return out;
		}

		/// This uses a separate voronoi container that is 'unshrunk' to get the prospective hull points.
		VoronoiDiagram::matrixType
			VoronoiDiagram::calcCandidateConvexHullPoints() const
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
				auto preVc = boost::shared_ptr<voro::pre_container>(new pre_container(
				mins(0)-1,maxs(0)+1,mins(1)-1,maxs(1)+1,mins(2)-1,maxs(2)+1,false,false,false));
			
				// Iterate over the precalced entries and insert only point that touch a boundary
				size_t numCells = 0;
				for (size_t i = 0; i < (size_t)cache["precalced"]->tblDoubles.rows(); ++i)
				//for (const auto &cell : *(cache["precalced"]->getCells()))
				{
					auto od = cache["precalced"]->tblDoubles.block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(i, 0);
					auto oi = cache["precalced"]->tblInts.block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(i, 0);

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
				
				
				boost::shared_ptr<voro::container> vcSmall(new container(
					mins(0)-1, maxs(0)+1, mins(1)-1, maxs(1)+1, mins(2)-1, maxs(2)+1,
					n_x, n_y, n_z, false, false, false, init_grid));
				
				preVc->setup(*vcSmall);

				// It's optional to use a pool here, since this Voronoi diagram is much smaller than the other.
				cache["precalcedSmall"] = boost::shared_ptr<CachedVoronoi>(new CachedVoronoi(numCells, vcSmall, mins, maxs));

				using namespace voro;
				boost::shared_ptr< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > out(
					new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(src->rows(), 4));
				out->setZero();

				// Check each particle to see if on the container surface
				size_t numSurfacePoints = 0;

				// Using the precalced loop here

				for (size_t i = 0; i < (size_t)cache["precalcedSmall"]->tblDoubles.rows(); ++i)
				//for (const auto &cell : *(precalcedSmall->getCells()))
				{
					auto od = cache["precalcedSmall"]->tblDoubles.block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(i, 0);
					auto oi = cache["precalcedSmall"]->tblInts.block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(i, 0);

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

		boost::shared_ptr<VoronoiDiagram> VoronoiDiagram::generateStandard(
			const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
			const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &points
			)
		{
			boost::shared_ptr<VoronoiDiagram> res(new VoronoiDiagram);
			res->mins = mins;
			res->maxs = maxs;
			res->src = boost::shared_ptr<const Eigen::Matrix<
				float, Eigen::Dynamic, Eigen::Dynamic> > (new 
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(points));
			return res;
		}

		boost::shared_ptr<VoronoiDiagram> VoronoiDiagram::loadHash(
			const std::string &hash)
		{
			boost::shared_ptr<VoronoiDiagram> res;

			using boost::filesystem::path;
			using boost::filesystem::exists;

			std::shared_ptr<registry::IOhandler> sh;
			std::shared_ptr<registry::IO_options> opts; // No need to set - it gets reset by findHashObj

			if (hashStore::findHashObj(hash, "voronoi.hdf5", sh, opts))
			{
				opts->setVal<std::string>("hash", hash);
				res = boost::shared_ptr<VoronoiDiagram>(new VoronoiDiagram);
				res->readMulti(sh, opts);
			}
			return res;
		}

		void VoronoiDiagram::writeToHash() const
		{
			using boost::filesystem::path;

			std::shared_ptr<registry::IOhandler> sh;
			std::shared_ptr<registry::IO_options> opts;

			if (_hash.lower == 0)
			{
				std::cerr << "Attempting to write voronoi diagram to hash, but hash is not set." << std::endl;
				return;
			}

			// Only store hash if a storage mechanism can be found
			if (hashStore::storeHash(_hash.string(), "voronoi.hdf5", sh, opts))
			{
				if (!rtmath::serialization::detect_compressed(opts->filename()))
					this->writeMulti(sh, opts);
			}
			else {
				std::cerr << "Cannot write voronoi diagram to hash " << _hash.string() << std::endl;
			}
		}

		boost::shared_ptr<VoronoiDiagram> VoronoiDiagram::loadHash(
			const HASH_t &hash)
		{
			return loadHash(boost::lexical_cast<std::string>(hash.lower));
		}



	}
}

