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
//#include <Voro/voro++.hh>
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

		template class usesDLLregistry <
			::rtmath::Voronoi::Voronoi_provider_registry,
			::rtmath::Voronoi::Voronoi_provider > ;
	}

	namespace Voronoi
	{
		Voronoi_provider::~Voronoi_provider() {}
		Voronoi_provider::Voronoi_provider() {}

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


		// These functions need knowledge about how to cast a new voronoi-based object from the plugins

		boost::shared_ptr<VoronoiDiagram> VoronoiDiagram::generateStandard(
			const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
			const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &points,
			const char* pluginid
			)
		{
			auto hooks = ::rtmath::registry::usesDLLregistry<Voronoi_provider_registry,
				Voronoi_provider >::getHooks();
			for (const auto &h : *(hooks.get()))
			{
				std::string pname(h.name), bname(pluginid);
				if (bname.size() && pname != bname) continue;
				if (!h.generator) continue;

				return h.generator(mins, maxs, points);
			}

			throw rtmath::debug::xUpcast("VoronoiDiagram", "generateStandard");
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

		boost::shared_ptr<VoronoiDiagram> VoronoiDiagram::generateBlank(const char* pluginid)
		{
			auto hooks = ::rtmath::registry::usesDLLregistry<Voronoi_provider_registry,
				Voronoi_provider >::getHooks();
			for (const auto &h : *(hooks.get()))
			{
				std::string pname(h.name), bname(pluginid);
				if (bname.size() && pname != bname) continue;
				if (!h.voronoiBlankGenerator) continue;

				return h.voronoiBlankGenerator();
			}

			throw rtmath::debug::xUpcast("VoronoiDiagram", "generateBlank");
		}

		// These methods exist to keep the Voronoi class from being pure virtual. If one is 
		// used, then it transparently reloads / upcasts the voronoi code to a plugin.

		/**
		* This function does something quite nasty. It replaces the current object, in the base class 
		* VoronoiDiagram, and replaces it with a derived object, found by querying the plugin database.
		**/
		void VoronoiDiagram::upcast(const char* pluginid) const
		{
			std::string bname(pluginid);
			if (!bname.size()) bname = pluginId;

			auto hooks = ::rtmath::registry::usesDLLregistry<Voronoi_provider_registry,
				Voronoi_provider >::getHooks();
			for (const auto &h : *(hooks.get()))
			{
				std::string pname(h.name);
				if (bname.size() && pname != bname) continue;
				if (!h.voronoiUpcastGenerator) continue;

				auto ptr = this->shared_from_this();
				// Remove const-ness
				auto mptr = boost::const_pointer_cast<VoronoiDiagram>(ptr);
				// Casting upwards to this plugin
				auto mres = h.voronoiUpcastGenerator(mptr);
				mptr.swap(mres); // Needs testing to check for stack issues!
				return;
			}
			
			throw rtmath::debug::xUpcast("VoronoiDiagram", "upcast");
		}





		void VoronoiDiagram::regenerateVoronoi() const
		{
			upcast(); regenerateVoronoi();
		}

		void VoronoiDiagram::regenerateFull() const
		{
			upcast(); regenerateFull();
		}

		const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* VoronoiDiagram::getCellMap() const
		{
			upcast(); return getCellMap();
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceDepth() const
		{
			upcast(); return calcSurfaceDepth();
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceDepthVectors() const
		{
			upcast(); return calcSurfaceDepthVectors();
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceNumNeighs() const
		{
			upcast(); return calcSurfaceNumNeighs();
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcSurfaceFillingOrder() const
		{
			upcast(); return calcSurfaceFillingOrder();
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcCandidateConvexHullPoints() const
		{
			upcast(); return calcCandidateConvexHullPoints();
		}

		VoronoiDiagram::matrixType VoronoiDiagram::calcPointsSAfracExternal() const
		{
			upcast(); return calcPointsSAfracExternal();
		}

		double VoronoiDiagram::surfaceArea() const
		{
			upcast(); return surfaceArea();
		}

		double VoronoiDiagram::volume() const
		{
			upcast(); return volume();
		}
	}
}

