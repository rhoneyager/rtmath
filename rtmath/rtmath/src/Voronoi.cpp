#include "Stdafx-voronoi.h"

#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <unordered_set>
#include <Voro++/voro++.hh>
#include "../rtmath/depGraph.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/error/error.h"

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
			std::vector<vertex> vertices(src->rows()); /// \todo Support serializing the vertices
			std::unordered_map<vertex*, size_t> vertexIdMap;
			vertexIdMap.reserve(src->rows());
			for (size_t i=0; i < (size_t) src->rows(); ++i)
			{
				//vertices[i] = boost::shared_ptr<vertex>(new vertex(true) );
				vertices[i].setOR(true);
				vertexIdMap[&vertices[i]] = i;
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

			// Construct the set of vertices from the vector
			rtmath::graphs::setVertex setVertices; //(vertices.begin(), vertices.end());
			//setVertices.reserve((size_t) src->rows());
			for (auto &v : vertices)
				setVertices.insert(&v);
			orderedVertex order;
			setVertex remaining;
			setVertex ignored;
			setVertex provided;

			auto initFilledPoints = calcCandidateConvexHullPoints();

			for (size_t i=0; i< (size_t) initFilledPoints.rows(); ++i)
				provided.insert(&vertices[(size_t) initFilledPoints(i, 3)]);

			graph g(setVertices);
			g.generate(provided, order, remaining, ignored);
			// Provided all have rank zero. order provides depth from the candidate surface cells.
			// Match the ordered vertices with their row
			for (auto &it : order)
			{
				const size_t &rank = it.second;
				auto IT = it.first;//.lock();
				size_t id = vertexIdMap.at(IT);
				out(id, 3) = (float) rank;
			}

			results["SurfaceDepthTrivial"] = std::move(out);
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
				std::vector<int> neigh; //,f_vert;
				//std::vector<double> v;
				c.neighbors(neigh);
				//c.face_vertices(f_vert);
				//c.vertices(crds(0),crds(1),crds(2),v);

				// Loop over all faces of the Voronoi cell
				// For faces that touch the walls, the neighbor number is negative
				for (auto &i : neigh)
				{
					if (i<0)
					{
						out(numSurfacePoints, 0) = (float) crds(0);
						out(numSurfacePoints, 1) = (float) crds(1);
						out(numSurfacePoints, 2) = (float) crds(2);
						out(numSurfacePoints, 3) = (float) id; // Initial point id
						numSurfacePoints++;
						break;
					}
				}
			} while (cl.inc());
			
			out.conservativeResize(numSurfacePoints, 4);
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

