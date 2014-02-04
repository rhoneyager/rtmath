#include "Stdafx-voronoi.h"

#include <cstdio>
#include <functional>
#include <scoped_allocator>
#include <boost/functional/hash.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_heap_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/flat_map.hpp>
#include <boost/interprocess/containers/flat_set.hpp>
#include <boost/interprocess/containers/set.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/unordered_set.hpp>
#include <Voro++/voro++.hh>
#include "../rtmath/depGraph.h"
#include "../rtmath/Voronoi/Voronoi.h"
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

	/*
	class AllocatorContainer : public boost::enable_shared_from_this<AllocatorContainer>
	{
		AllocatorContainer() {}
	public:
		static boost::shared_ptr<AllocatorContainer> generate()
		{
		}

	};
	*/
}


namespace rtmath
{
	namespace Voronoi
	{
		/// Persistent internal Voronoi cell storage object
		class CachedVoronoiCellBase
		{
		protected:
			void baseInit()
			{
				id = 0;
				r = 0;
				sa_full = 0;
				sa_ext = 0;
				vol = 0;
				max_radius_squared = 0;
				total_edge_distance = 0;
				nFaces = 0;
				nEdges = 0;
				pos.setZero();
			}
			CachedVoronoiCellBase() {baseInit();}
			virtual ~CachedVoronoiCellBase() {}
		public:
			/// Particle id
			int id;
			/// Particle radius (unused)
			double r;
			/// Surface area of entire Voronoi cell
			double sa_full;
			/// Surface area of the cell that does not touch another cell
			double sa_ext;
			/// Particle volume
			double vol;
			/// Maximum radius squared from particle centroid
			double max_radius_squared;
			/// Sum of all edge lengths
			double total_edge_distance;
			/// Number of faces and edges in the particle
			int nFaces, nEdges;
			/// Centroid of particle (uncertain of coordinate system)
			Eigen::Matrix3d centroid;
			/// Position vector of particle
			Eigen::Matrix3d pos;
			/// Convenience function to see if the particle touches the 'surface'
			bool isSurface() const { if (sa_ext) return true; return false; }
		};

		/// \brief Internal aligned class for persistent voronoi cell information storage
		/// \note MSVC 2012 has a bug in object construction at runtime. 
		/// It seems related to passing constructor arguments with two allocators / using the 
		/// segment manager twice in constructors. As such, I am using static arrays
		template <class AllocInt = std::allocator<int>, class AllocDouble = std::allocator<double> >
		class CachedVoronoiCell : public CachedVoronoiCellBase
		{
		private:
			// Cannot store these because it causes annoying errors when generating operator=.
			//AllocInt allocInt;
			//AllocDouble allocDouble;
		public:
			static const size_t ArraySize = 50;
			virtual ~CachedVoronoiCell() {}
#ifndef _MSC_FULL_VER
			CachedVoronoiCell(const AllocInt& allocInt = AllocInt(), const AllocDouble& allocDouble = AllocDouble())
				: neigh(allocInt), f_vert(allocInt), f_areas(allocDouble) {}
			CachedVoronoiCell(voro::voronoicell_neighbor &vc, 
				const AllocInt& allocInt = AllocInt(), const AllocDouble& allocDouble = AllocDouble())
				: neigh(allocInt), f_vert(allocInt), f_areas(allocDouble)
			{ calc(vc); }
#endif
			CachedVoronoiCell()
			{ }
			CachedVoronoiCell(voro::voronoicell_neighbor &vc)
			{ calc(vc); }
			/// Cell neighbor and vertex lists. The integer in neigh 
			// corresponds to the cell id in CachedVoronoi.
			//boost::interprocess::vector<int, AllocInt> neigh, f_vert;
			std::array<int, ArraySize> neigh, f_vert;
			/// Areas of each face
			//boost::interprocess::vector<double, AllocDouble> f_areas;
			std::array<double, ArraySize> f_areas;
			//std::vector<double> v;
			/// \note Position information must be set separately (not in vc)
			void calc(voro::voronoicell_neighbor &vc)
			{
				Eigen::Matrix3d crds;
				// Need to copy vectors due to different allocators
				std::vector<int> lneigh, lf_vert;
				std::vector<double> lf_areas;
				vc.neighbors(lneigh);
				vc.face_vertices(lf_vert);
				//vc.vertices(crds(0),crds(1),crds(2),v);
				vc.face_areas(lf_areas);

				//neigh.resize(lneigh.size());
				
				std::copy_n(lneigh.begin(), std::min( ArraySize, lneigh.size()) , neigh.begin());
				//f_vert.resize(lf_vert.size());
				std::copy_n(lf_vert.begin(), std::min( ArraySize, lf_vert.size()) , f_vert.begin());
				//f_areas.resize(lf_areas.size());
				std::copy_n(lf_areas.begin(), std::min( ArraySize, lf_areas.size()) , f_areas.begin());

				vol = vc.volume();
				sa_full = vc.surface_area();
				nFaces = vc.number_of_faces();
				nEdges = vc.number_of_edges();
				vc.centroid(centroid(0), centroid(1), centroid(2));
				
				max_radius_squared = vc.max_radius_squared();
				total_edge_distance = vc.total_edge_distance();

				for (int i=0; i < neigh.size(); ++i)
					if (neigh[i]<=0)
						sa_ext += f_areas[i];
			}
		};

		/// Storage container class for the cached Voronoi cell information
		class CachedVoronoi
		{
			friend class VoronoiDiagram;
		private:
			mutable boost::interprocess::managed_heap_memory m;
			mutable double sa, vol;
		public:
			typedef boost::interprocess::allocator<int, boost::interprocess::managed_heap_memory::segment_manager> IntAllocator;
			typedef boost::interprocess::allocator<double, boost::interprocess::managed_heap_memory::segment_manager> DoubleAllocator;
			typedef boost::interprocess::allocator<CachedVoronoiCell<IntAllocator, DoubleAllocator>, 
				boost::interprocess::managed_heap_memory::segment_manager> CachedVoronoiCellAllocator;
			const IntAllocator intAllocator;
			const DoubleAllocator doubleAllocator;
			const CachedVoronoiCellAllocator cachedVoronoiCellAllocator;

			mutable boost::shared_ptr<voro::container> vc;
		private:
			mutable boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator>, 
				CachedVoronoiCellAllocator> *c;
		public:

			CachedVoronoi(size_t numPoints, boost::shared_ptr<voro::container> vc) : 
				c(nullptr),
				vc(vc),
				m(10*1024*numPoints), // 10 kb per point should be enough for point lists + vertices
				intAllocator(m.get_segment_manager()),
				doubleAllocator(m.get_segment_manager()), 
				cachedVoronoiCellAllocator(m.get_segment_manager()),
				sa(0),
				vol(0)
			{
				if (vc) regenerateCache(numPoints);
			}
			~CachedVoronoi()
			{
			}
			void regenerateCache(size_t numPoints) const
			{
				if (!vc) return;
				// Iterate over cells and store cell information 
				// (prevents constant recalculations)
				using namespace boost::interprocess;

				c = m.find_or_construct<boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator>, 
					CachedVoronoiCellAllocator> >("cells")(cachedVoronoiCellAllocator);
				// Cannot just use resize(numPoints) here because the appropriate allocators must be specified.
				if (c->size() != numPoints)
					c->resize(numPoints
						//, CachedVoronoiCell<IntAllocator, DoubleAllocator>()
						//(m)
						//(intAllocator, doubleAllocator)
						);

				using namespace voro;
				voronoicell_neighbor n;
				c_loop_all cl(*(vc.get()));
				if (cl.start()) do if (vc->compute_cell(n,cl)) {
					// Quantities explicitly retrieved this way to avoid any c->at(id) potential issues.
					int id;
					Eigen::Matrix3d pos;
					double r;
					cl.pos(id, pos(0), pos(1), pos(2), r); // getting id directly into field would be problematic
					
					auto &ci = c->at(id);

					ci.id = id;
					ci.r = r;
					ci.pos = pos;

					if (id % 1000 == 0) std::cerr << id << "\n";
					ci.calc(n);

					// Set a few fields for convenience
					vol += ci.vol;
					sa += ci.sa_ext;
				} while (cl.inc());
			}
			
			
			/// Calculate the surface area of the bulk figure
			double surfaceArea() const { return sa; }
			/// Calculate the volume of the bulk figure
			double volume() const { return vol; }
			/// Get pointer to the set of stored voronoi cells
			boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator>, 
				CachedVoronoiCellAllocator>* getCells() const
			{
				return c;
				//return m.find<boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator> > >("cells").first;
			}
		};



		VoronoiDiagram::VoronoiDiagram() {}

		void VoronoiDiagram::setHash(HASH_t hash) { this->hash = hash; }

		double VoronoiDiagram::surfaceArea() const
		{
			if (!precalced) return 0;
			return precalced->surfaceArea();
		}

		double VoronoiDiagram::volume() const
		{
			if (!precalced) return 0;
			return precalced->volume();
		}

		/// \note This regenerates the REGULAR diagram, with surface fitting
		void VoronoiDiagram::regenerateVoronoi() const
		{
			if (vc) return;
			// Set up the number of blocks that the container is divided into
			const int n_x=50,n_y=50,n_z=50, init_grid=150;
			using namespace voro;
			vc = boost::shared_ptr<voro::container>(new container(
				mins(0),maxs(0),mins(1),maxs(1),mins(2),maxs(2),
				n_x,n_y,n_z,false,false,false,init_grid));

			//wall_initial_shape wis;
			vc->add_wall(wis);

			// Add particles into the container
			for (size_t i=0; i < (size_t) src->rows(); ++i)
			{
				auto pt = src->block<1, 3>(i, 0);
				vc->put((int) i, pt(0), pt(1), pt(2));
			}
		}

		/** This function iterates over the entire diagram, generating the data for all 
		* of the cells, with regards to cell ids, voronoi centers and neighbors.
		* As needed, will add information about cell vertices and faces.
		* This is needed to prevent repetitive looping over the Voronoi structure.
		**/
		void VoronoiDiagram::regenerateFull() const
		{
			if (precalced) return;
			if (!vc) regenerateVoronoi();
			precalced = boost::shared_ptr<CachedVoronoi>(new CachedVoronoi((size_t) src->rows(), vc));
		}

		boost::shared_ptr< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > VoronoiDiagram::calcSurfaceDepth() const
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
			// Using depGraph and the initial Candidate Convex Hull points

			// Construct the dependency graph
			using namespace rtmath::graphs;
			/// \todo Support exporting the vertices (with connection information) using 
			/// serialization and graphical (i.e. silo) output.
			std::vector<vertex> vertices((size_t) src->rows());

			using namespace boost::interprocess;

			//mapped_region region(anonymous_shared_memory(1024*1024*128));
			managed_heap_memory &m = precalced->m;
			typedef allocator<std::pair<const vertex*, size_t>, managed_heap_memory::segment_manager>
				PairAllocator;
			const PairAllocator pairAllocator(m.get_segment_manager());
			typedef boost::interprocess::flat_map<vertex*, size_t, std::less<const vertex*>,
				PairAllocator> vIdMap;
			vIdMap *vertexIdMap = m.construct<vIdMap>("SurfaceDepth_vertexIdMap")
				(std::less<const vertex*>(), pairAllocator);
			//std::unordered_map<vertex*, size_t, std::hash<vertex*>, std::equal_to<vertex*>,
			//	boost::pool_allocator<std::pair<const vertex*, size_t> > > vertexIdMap;

			vertexIdMap->reserve(src->rows());
			for (size_t i=0; i < (size_t) src->rows(); ++i)
			{
				//vertices[i] = boost::shared_ptr<vertex>(new vertex(true) );
				vertices[i].setOR(true);
				(*vertexIdMap)[&vertices[i]] = i;
			}
			
			// Iterate over the precalced entries and insert only point that touch a boundary
			for (const auto &cell : *(precalced->getCells()))
			{
				const int &id = cell.id;
				if (id % 1000 == 0) std::cerr << id << "\n";

				for (auto &i : cell.neigh)
				{
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
			VertexAllocator vAllocator (m.get_segment_manager());

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

			auto initFilledPoints = calcCandidateConvexHullPoints();

			for (size_t i=0; i< (size_t) initFilledPoints->rows(); ++i)
				provided.insert(&vertices[(size_t) (*initFilledPoints)(i, 3)]);

			generateGraph<bSetVertex, orderedVertex>::generate(
				setVertices, provided, order, remaining, ignored);

			//graph g(setVertices);
			//g.generate(provided, order, remaining, ignored);

			// Provided all have rank zero. order provides depth from the candidate surface cells.
			// Match the ordered vertices with their row
			for (auto &it : order)
			{
				const size_t &rank = it.second;
				auto IT = it.first;//.lock();
				size_t id = vertexIdMap->at(IT);
				(*out)(id, 3) = (float) rank;
			}

			// Clean up vertex graph
			m.destroy_ptr(vertexIdMap);

			results["SurfaceDepth"] = out;
			return out;
		}

		/// This uses a separate voronoi container that is 'unshrunk' to get the prospective hull points.
		boost::shared_ptr< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
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
			const int n_x=50,n_y=50,n_z=50, init_grid=150;
			using namespace voro;
			boost::shared_ptr<voro::container> vcSmall(new container(
				mins(0),maxs(0),mins(1),maxs(1),mins(2),maxs(2),
				n_x,n_y,n_z,false,false,false,init_grid));
			// Iterate over the precalced entries and insert only point that touch a boundary
			size_t numCells = 0;
			for (const auto &cell : *(precalced->getCells()))
			{
				if (cell.isSurface())
				{
					vcSmall->put((int) numCells, cell.pos(0), cell.pos(1), cell.pos(2));
					numCells++;
				}
			}

			// It's optional to use a pool here, since this Voronoi diagram is much smaller than the other.
			boost::shared_ptr<CachedVoronoi> precalcedSmall(new CachedVoronoi(numCells, vcSmall));

			using namespace voro;
			boost::shared_ptr< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > out(
				new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(src->rows(), 4));
			out->setZero();

			// Check each particle to see if on the container surface
			size_t numSurfacePoints = 0;

			// Using the precalced loop here
			for (const auto &cell : *(precalcedSmall->getCells()))
			{
				if (cell.isSurface())
				{
					(*out)(numSurfacePoints, 0) = (float) cell.pos(0);
					(*out)(numSurfacePoints, 1) = (float) cell.pos(1);
					(*out)(numSurfacePoints, 2) = (float) cell.pos(2);
					(*out)(numSurfacePoints, 3) = (float) cell.id; // Initial point id
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
			Eigen::Array3f &mins, Eigen::Array3f &maxs,
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> points
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


	}
}

