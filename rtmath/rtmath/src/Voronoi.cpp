#include "Stdafx-voronoi.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <functional>
#include <random>
#include <boost/functional/hash.hpp>
//#include <boost/pool/pool.hpp>
//#include <boost/pool/pool_alloc.hpp>
//#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_heap_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/flat_map.hpp>
#include <boost/interprocess/containers/flat_set.hpp>
#include <boost/interprocess/containers/set.hpp>
//#include <boost/interprocess/containers/
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>
//#include <unordered_map>
//#include <unordered_set>
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

	/// \brief Draws a ploygon in POV-ray format
	/// \note Taken from voro++ ploygons example, with c-style io translated to c++-style
	void drawPOV_polygon(std::ostream &out, std::vector<int> &f_vert, std::vector<double> &v,int j)
	{
		std::vector<std::string> s(600);
		int k,l,n=f_vert[j];

		// Create POV-Ray vector strings for each of the vertices
		for(k=0;k<n;k++) {
			l=3*f_vert[j+k+1];
			std::ostringstream o;
			o << "<" << v[l] << "," << v[l+1] << "," << v[l+2] << ">";
			s[k] = o.str();
		}

		// Draw the interior of the polygon
		out << "union{\n";
		for(k=2;k<n;k++) 
			out << "\ttriangle{" << s[0] << "," << s[k-1] << "," << s[k] << "}\n";
		out << "\ttexture{t1}\n}\n";

		// Draw the outline of the polygon
		out << "union{\n";
		for(k=0;k<n;k++) {
			l=(k+1)%n;
			out << "\tcylinder{" << s[k] << "," << s[l] << ",r}\n\tsphere{" << s[l] << ",r}\n";
		}
		out << "\ttexture{t2}\n}\n";
	}
}


namespace rtmath
{
	namespace Voronoi
	{
		VoronoiDiagram::VoronoiDiagram() {}

		void VoronoiDiagram::setHash(HASH_t hash) { this->hash = hash; }

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

		}

		const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VoronoiDiagram::calcSurfaceDepthTrivial() const
		{
			if (results.count("SurfaceDepthTrivial"))
			{
				return results.at("SurfaceDepthTrivial");
			}
			regenerateVoronoi();

			using namespace voro;
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> out;
			out = *src;
			out.conservativeResize(src->rows(), 4);
			out.col(3) = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(src->rows(), 1);
			// Using depGraph and the initial Candidate Convex Hull points
			
			// Construct the dependency graph
			using namespace rtmath::graphs;
			std::vector<vertex> vertices((size_t) src->rows()); /// \todo Support serializing the vertices

			using namespace boost::interprocess;

			//mapped_region region(anonymous_shared_memory(1024*1024*128));
			managed_heap_memory m(1024*1024*128);
			typedef allocator<std::pair<const vertex*, size_t>, managed_heap_memory::segment_manager>
				PairAllocator;
			const PairAllocator pairAllocator(m.get_segment_manager());
			typedef boost::interprocess::flat_map<vertex*, size_t, std::less<const vertex*>,
				PairAllocator> vIdMap;
			vIdMap *vertexIdMap = m.construct<vIdMap>("vertexIdMap")
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

			//double vol = vc->sum_cell_volumes();

			//particle_order po;
			voronoicell_neighbor c;
			c_loop_all cl(*(vc.get()));
			if (cl.start()) do if (vc->compute_cell(c,cl)) {
				//Eigen::Matrix3d crds;
				//cl.pos(crds(0),crds(1),crds(2));
				int id = cl.pid(); // Particle id as specified in voronoi cell construction!
				if (id % 1000 == 0) std::cerr << id << "\n";
				std::vector<int> neigh,f_vert;
				std::vector<double> v;
				c.neighbors(neigh);
				//c.face_vertices(f_vert);
				//c.vertices(crds(0),crds(1),crds(2),v);

				// Loop over all faces of the Voronoi cell
				// For faces that touch the walls, the neighbor number is negative
				for (auto &i : neigh)
				{
					if (i<0) continue;
					auto distsq = [&](size_t i, size_t j) -> float
					{
						float res = (out.block(i,0,1,3) - out.block(j,0,1,3)).norm();
						return res;
					};
					
					if (distsq(i, id) < 2.2f)
						vertices[id].addSlot(&vertices[i]);
				}
			} while (cl.inc());

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

			for (size_t i=0; i< (size_t) initFilledPoints.rows(); ++i)
				provided.insert(&vertices[(size_t) initFilledPoints(i, 3)]);

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
				out(id, 3) = (float) rank;
			}

			results["SurfaceDepthTrivial"] = std::move(out);

			// Free the dependency graph table
			//boost::singleton_pool<boost::pool_allocator_tag, sizeof(vertex*)>::release_memory();

			return results.at("SurfaceDepthTrivial");
		}

		void VoronoiDiagram::calcSurfaceDepth(
			Eigen::Matrix<float, Eigen::Dynamic, 4> &out) const
		{
			throw;
		}

		const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& 
			VoronoiDiagram::calcCandidateConvexHullPoints() const
		{
			if (results.count("CandidateConvexHullPoints"))
			{
				return results.at("CandidateConvexHullPoints");
			}
			regenerateVoronoi();

			// Test output to show cell boundaries for the hull
			std::ofstream oCandidates("CandidateConvexHullPoints_extfaces.pov");
			//std::default_random_engine generator;
			//std::binomial_distribution<int> distribution(1, 0.01);
			//std::discrete_distribution<int> distribution(2,0,1,[](double d){if (d>0.1) return 1; return 2500; });
			std::ofstream oint("CandidateConvexHullPoints_intfaces.pov");

			using namespace voro;
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> out;
			out.resize(src->rows(), 4);

			// Check each particle to see if on the container surface
			size_t numSurfacePoints = 0;

			voronoicell_neighbor c;
			c_loop_all cl(*(vc.get()));
			if (cl.start()) do if (vc->compute_cell(c,cl)) {
				Eigen::Matrix3d crds;
				cl.pos(crds(0),crds(1),crds(2));
				int id = cl.pid();
				if (id % 1000 == 0) std::cerr << id << "\n";
				std::vector<int> neigh, f_vert;
				std::vector<double> v;
				c.neighbors(neigh);
				c.face_vertices(f_vert);
				c.vertices(crds(0),crds(1),crds(2),v);

				// Randomly select every 500th cell and write out the Voronoi boundaries
				//bool writeBounds = false;
				//if (distribution(generator) > 0)
				//{
				//	std::cerr << "Writing bounds for cell " << id << "\n";
				//	writeBounds = true;
				//}

				// Loop over all faces of the Voronoi cell
				// For faces that touch the walls, the neighbor number is negative
				for (int i=0, j=0; i < neigh.size(); ++i)
				//for (auto &i : neigh)
				{
					bool hasSfc = false;
					if (neigh[i]<=0)
					{
						out(numSurfacePoints, 0) = (float) crds(0);
						out(numSurfacePoints, 1) = (float) crds(1);
						out(numSurfacePoints, 2) = (float) crds(2);
						out(numSurfacePoints, 3) = (float) id; // Initial point id
						if (!hasSfc)
						{
							numSurfacePoints++;
							hasSfc = true;
						}

						drawPOV_polygon(oCandidates, f_vert, v, j);
						//break; // from surface point detection
					}
					//else drawPOV_polygon(oint, f_vert, v, j);
					//if (writeBounds) drawPOV_polygon(oint, f_vert, v, j);
					//j+=f_vert[j]+1;
				}
			} while (cl.inc());
			
			out.conservativeResize(numSurfacePoints, 4);

			vc->draw_particles_pov("CandidateConvexHullPoints_p.pov");
			vc->draw_cells_pov("CandidateConvexHullPoints_v.pov");
			//vc->draw_particles("CandidateConvexHullPoints_p.gnu");
			//vc->draw_cells_gnuplot("CandidateConvexHullPoints_v.gnu");

			results["CandidateConvexHullPoints"] = std::move(out);
			return results.at("CandidateConvexHullPoints");
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

