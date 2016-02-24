#pragma once
#include "../defs.h"
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/io.h>

namespace rtmath {
	namespace Voronoi {
		class VoronoiDiagram;
	}
	namespace ddscat {
		namespace shapefile {
			class shapefile;
			class shapefile_IO_input_registry {};
			class shapefile_IO_output_registry {};
			//class shapefile_serialization {};
			class shapefile_Standard {};
			class shapefile_query_registry {};

			/// \brief This class is used for plugins to register themselves to handle shape queries.
			struct DLEXPORT_rtmath_ddscat shapefile_db_registry
			{
				struct DLEXPORT_rtmath_ddscat shapefile_db_comp {
					bool operator() (const std::shared_ptr<const shapefile>& lhs,
						const std::shared_ptr<const shapefile>& rhs) const;
					bool operator() (const boost::shared_ptr<const shapefile>& lhs,
						const boost::shared_ptr<const shapefile>& rhs) const;
				};

				/// Language-Integrated Query (LINQ) is not a good idea here, since an external database is used
				class DLEXPORT_rtmath_ddscat shapefile_index :
					virtual public ::Ryan_Debug::registry::collectionTyped < shapefile > //,
					//virtual public ::rtmath::registry::collectionTyped<::rtmath::Voronoi::VoronoiDiagram>
				{
					shapefile_index();
				public:
					std::set<std::string> hashLowers, hashUppers,
						flakeTypes, refHashLowers;
					//std::map<float, float > standardDs;
					Ryan_Debug::splitSet::intervals<float> dipoleSpacings;
					std::map<std::string, std::string> tags;
					//std::vector<std::pair<size_t, size_t> > dipoleRanges;
					Ryan_Debug::splitSet::intervals<size_t> dipoleNumbers;
				public:
					~shapefile_index();
					static std::shared_ptr<shapefile_index> generate();

					typedef std::shared_ptr<std::set<boost::shared_ptr<const shapefile>, shapefile_db_comp > > collection;
					std::pair<collection, std::shared_ptr<Ryan_Debug::registry::DBhandler> >
						doQuery(std::shared_ptr<Ryan_Debug::registry::DBhandler> = nullptr,
						std::shared_ptr<Ryan_Debug::registry::DB_options> = nullptr) const;

					virtual bool filter(const shapefile*) const override;
					//virtual bool filter(const ::rtmath::Voronoi::VoronoiDiagram*) const override;

					/**
					* \brief Add support for filtering based on existing, loaded objects (in a collection).
					*
					* Will pull information from the database for filling.
					* \param srcs is a preexisting collection of loaded objects
					* \param doUnion indicates whether the database is used to merely add tag
					*			information to the already-loaded objects, or whether objects in the
					*			database that match the criteria are also added in.
					* \param doDb indicates whether the database is consulted for the lookup. If not,
					* only filter the objects in srcs.
					**/
					std::pair<collection, std::shared_ptr<Ryan_Debug::registry::DBhandler> >
						doQuery(collection srcs,
						bool doUnion = false, bool doDb = true,
						std::shared_ptr<Ryan_Debug::registry::DBhandler> = nullptr,
						std::shared_ptr<Ryan_Debug::registry::DB_options> = nullptr) const;
				};

				shapefile_db_registry();
				virtual ~shapefile_db_registry();
				/// Module name.
				const char* name;

				enum class updateType { INSERT_ONLY, UPDATE_ONLY, INSERT_AND_UPDATE };

				/// \todo As more database types become prevalent, move this over to 
				/// rtmath::registry and standardize.
				typedef std::function < std::shared_ptr<Ryan_Debug::registry::DBhandler>
					(const shapefile_index&, shapefile_index::collection,
					std::shared_ptr<Ryan_Debug::registry::DBhandler>, std::shared_ptr<Ryan_Debug::registry::DB_options>) > queryType;
				typedef std::function < std::shared_ptr<Ryan_Debug::registry::DBhandler>
					(const shapefile_index::collection, updateType,
					std::shared_ptr<Ryan_Debug::registry::DBhandler>, std::shared_ptr<Ryan_Debug::registry::DB_options>) > writeType;
				typedef std::function < bool(std::shared_ptr<Ryan_Debug::registry::DBhandler>,
					std::shared_ptr<Ryan_Debug::registry::DB_options>) > matchType;

				/// Get cross-sections from small stats
				queryType fQuery;
				/// Get pfs from small stats
				writeType fInsertUpdate;

				matchType fMatches;
			};
		}
	}
}
namespace Ryan_Debug {
	namespace registry {

		extern template struct IO_class_registry_writer <
			::rtmath::ddscat::shapefile::shapefile > ;

		extern template struct IO_class_registry_reader <
			::rtmath::ddscat::shapefile::shapefile > ;

		extern template class usesDLLregistry <
			::rtmath::ddscat::shapefile::shapefile_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::shapefile::shapefile> > ;
		//::rtmath::ddscat::shapefile::shapefile_IO_class_registry>;

		extern template class usesDLLregistry <
			::rtmath::ddscat::shapefile::shapefile_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::shapefile::shapefile> > ;

		extern template class usesDLLregistry <
			::rtmath::ddscat::shapefile::shapefile_query_registry,
			::rtmath::ddscat::shapefile::shapefile_db_registry > ;
	}
	namespace io {
		template <>
		DLEXPORT_rtmath_ddscat boost::shared_ptr
			<::rtmath::ddscat::shapefile::shapefile> customGenerator();
	}
}
namespace rtmath {
	namespace ddscat {

		namespace stats {
			class shapeFileStatsBase;
			class shapeFileStats;
		}
		class convexHull;

		/// Contains everything to do with low-level manipulation of shape files.
		namespace shapefile {
			struct convolutionCellInfo;
			class shapefile;

			/// Provides local readers and writers for ddscat ddpar data (it's a binder)
			class DLEXPORT_rtmath_ddscat implementsDDSHP :
				private Ryan_Debug::io::implementsIObasic<shapefile, shapefile_IO_output_registry,
				shapefile_IO_input_registry, shapefile_Standard>
			{
			public:
				virtual ~implementsDDSHP() {}
			protected:
				implementsDDSHP();
			private:
				static const std::set<std::string>& known_formats();
			};
			
			/// Class for reading / writing shapefiles. May be used in statistical calculations.
			class DLEXPORT_rtmath_ddscat shapefile : 
				virtual public boost::enable_shared_from_this<shapefile>,
				virtual public ::Ryan_Debug::registry::usesDLLregistry<
					::rtmath::ddscat::shapefile::shapefile_IO_input_registry, 
					::Ryan_Debug::registry::IO_class_registry_reader<::rtmath::ddscat::shapefile::shapefile> >,
					virtual public ::Ryan_Debug::registry::usesDLLregistry<
					::rtmath::ddscat::shapefile::shapefile_IO_output_registry, 
					::Ryan_Debug::registry::IO_class_registry_writer<::rtmath::ddscat::shapefile::shapefile> >,
					virtual public ::Ryan_Debug::io::implementsStandardWriter<shapefile, shapefile_IO_output_registry>,
					virtual public ::Ryan_Debug::io::implementsStandardReader<shapefile, shapefile_IO_input_registry>,
					virtual public ::Ryan_Debug::registry::usesDLLregistry<
					shapefile_query_registry, shapefile_db_registry >,
				virtual public implementsDDSHP,
				virtual public ::Ryan_Debug::io::implementsDBbasic<shapefile, shapefile_db_registry,
					shapefile_db_registry::shapefile_index, 
					shapefile_db_registry::shapefile_db_comp, shapefile_query_registry>
				//virtual public ::rtmath::io::Serialization::implementsSerialization<
				//	shapefile, shapefile_IO_output_registry, shapefile_IO_input_registry, shapefile_serialization>,
			{
				// Need readVector as a friend class
				friend boost::shared_ptr<shapefile> Ryan_Debug::io::customGenerator<shapefile>();
				//shapefile(const std::string &filename); // Handled by the generator
				shapefile(std::istream &in);
				shapefile();
			public:
				virtual ~shapefile();
				static boost::shared_ptr<shapefile> generate(const std::string &filename);
				static boost::shared_ptr<shapefile> generate(std::istream &in);
				static boost::shared_ptr<shapefile> generate();
				/// Copy from an existing object
				static boost::shared_ptr<shapefile> generate(boost::shared_ptr<const shapefile>);
				
				/// \todo Fix the shapefile inheritance copy constructor.
				/// Problem is caused by io.h implementsIObasic inF being a reference.
//#if _MSC_FULL_VER
				shapefile& operator=(const shapefile&);
//#endif

				bool operator<(const shapefile &) const;
				bool operator==(const shapefile &) const;
				bool operator!=(const shapefile &) const;

				/// Function to output shapefile information
				void info(std::ostream &out = std::cout) const;

				/// Function to fix the shape center of mass to match calculated stats
				void fixStats();
				/// Write ddscat-formatted shapefile to the given output stream.
				void print(std::ostream &out = std::cout) const;
				/// Resizes arrays to hold the desired number of points
				void resize(size_t num);
				/// Read shape dipoles from a memory buffer
				void readContents(const char *in, size_t headerEnd);
				/// Read in ONLY a shape header (for speed with dipole matching) - string, NOT a filename
				void readHeaderOnly(const std::string &str);
				static void readDDSCAT(boost::shared_ptr<shapefile>, std::istream&, std::shared_ptr<Ryan_Debug::registry::IO_options>);
				/// Write shape to the hash directory (convenience function)
				void writeToHash() const;
				/// Write a standard DDSCAT shapefile to a stream (no compression)
				static void writeDDSCAT(const boost::shared_ptr<const shapefile>, std::ostream &, std::shared_ptr<Ryan_Debug::registry::IO_options>);
				/// Function type definition for a function that determines a decimated cell refractive index.
				typedef std::function < size_t(const convolutionCellInfo&) > decimationFunction;

				/** \brief Decimate a shapefile
				* This version of the function examines the number of dipoles in a given dx*dy*dz
				* unit paralelipipet, and then constructs a smaller shapefile object with the matching parameters.
				*
				* \param dFunc specifies a decimation function that determines the decimated cell's dielectric.
				**/
				boost::shared_ptr<shapefile> decimate(size_t dx = 2, size_t dy = 2, size_t dz = 2,
					decimationFunction dFunc = shapefile::decimateDielCount) const;

				/// \brief Convenience function to decimate using the same degree in each dimension
				inline boost::shared_ptr<shapefile> decimate(size_t degree = 2) const { return decimate(degree, degree, degree); }

				/** \brief Upscale a shapefile
				* This function takes each dipole and multiplies it into a rectangular cell of a given size.
				*
				* All refractive indices are the same as the initial dipole.
				**/
				boost::shared_ptr<shapefile> enhance(size_t dx = 2, size_t dy = 2, size_t dz = 2) const;
				/// \brief Convenience function to upscale using the same degree in each dimension
				inline boost::shared_ptr<shapefile> enhance(size_t d = 2) const { return enhance(d, d, d); }

				/// \brief Decimation dielectric function that assigns a dielectric
				/// that corresponds to the number of filled dipoles.
				static size_t decimateDielCount(const convolutionCellInfo&);

				/// \brief Decimation dielectric function that fills a dielectric 
				/// based on a threshold value (high-pass, inclusive).
				static size_t decimateThreshold(const convolutionCellInfo&, size_t threshold);

				/** \brief Get filled cells within a certain distance
				*
				* \param rsq is the radius squared for the search
				* \param out is the output vector that holds the cell indices
				* \param x,y,s are the coordinates of the search cell
				**/
				//void getNeighbors(float x, float y, float z, float rsq, std::vector<size_t>& out) const;

				/** \brief Get filled cells within a certain distance
				*
				* \param rsq is the radius squared for the search
				* \param out is the output vector that holds the cell indices
				* \param index is the cell lattice point index
				**/
				//void getNeighbors(size_t index, float rsq, std::vector<size_t>& out) const;


			private:
				/// Read a shapefile from an uncompressed string
				void readString(const std::string &in, bool headerOnly = false);
				void _init();
				void readHeader(const char *in, size_t &headerEnd);
				/// Recalculate stats after a manipulation operation
				void recalcStats();
				mutable Ryan_Debug::hash::HASH_t _localhash;
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW

				/// Original filename
				std::string filename;
				/// When timported
				std::string ingest_timestamp;
				/// The system that was imported on
				std::string ingest_hostname;
				/// The user account that imported the shape
				std::string ingest_username;
				/// Revision of the rtmath code for ingest
				int ingest_rtmath_version;
				/// Standard dipole spacing for this shape (the value usually used)
				float standardD;

				//std::vector<Eigen::Vector3f> 
				/// \todo Move latticePts and the rest into const shared_ptr containers
				Eigen::Matrix<float, Eigen::Dynamic, 3>
					latticePts, // Untransformed points
					latticePtsStd, // Points with coord translation based on file properties
					latticePtsNorm, // Points with coord transform to mean center of shape
					latticePtsRi; // Dielectric information
				/// The first field in the point listings. Provides a nonunique point id.
				Eigen::Matrix<int, Eigen::Dynamic, 1> latticeIndex;
				/// Tags used to describe the shape (decimation, perturbations, ...). Not saved in .shp format.
				std::map<std::string, std::string> tags;
				/**
				 * Container for other, temporary tables, which reflect different information 
				 * about the shapefile, such as number of dipoles from the surface, the point's 
				 * contribution to the moment of inertia, or other composition classifiers.
				 *
				 * These tables are not saved in the standard shapefile format, though they may 
				 * be serialized. The bov format should write them out.
				 **/
				mutable std::map < std::string, boost::shared_ptr<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > latticeExtras;

				/// Container for temporary Voronoi diagrams (disabled due to high memory usage)
				//mutable std::map < std::string, boost::shared_ptr<Voronoi::VoronoiDiagram> > voronoi_diagrams;

				/// Convenient function override to generate a given Voronoi diagram uniquely
				boost::shared_ptr<Voronoi::VoronoiDiagram> generateVoronoi(
					const std::string &name,
					std::function < boost::shared_ptr<Voronoi::VoronoiDiagram>(
					const Eigen::Array3f&, const Eigen::Array3f&,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>&, const char*)>) const;

				size_t numPoints;
				std::set<size_t> Dielectrics;
				std::string desc;
				/// Calculates the hash of the given shapefile. Used as a reference when 
				/// serializing the shape. The hash table allows for smaller stats files.
				Ryan_Debug::hash::HASH_t hash() const;
				/// Forcibly set the hash of the shapefile
				void setHash(const Ryan_Debug::hash::HASH_t &);
				/// Force a hash to be recalculated
				Ryan_Debug::hash::HASH_t rehash() const;
				// Specified in shape.dat
				// a1 and a2 are the INITIAL vectors (before rotation!)
				// usually a1 = x_lf, a2 = y_lf
				// choice of a1 and a2 can reorient the shape (useful for KE, PE constraints)
				Eigen::Array3f a1, a2, a3, d, x0, xd;

				// These are RAW values (no mean or d scaling)
				Eigen::Array3f mins, maxs, means;

				//boost::shared_ptr< rtmath::Garrett::pointContainer > _pclObj;

				friend class ::rtmath::ddscat::stats::shapeFileStatsBase;
				friend class ::rtmath::ddscat::stats::shapeFileStats;
				friend class ::rtmath::ddscat::convexHull;

				/// Convenience functions to load shape based on hash
				/// \throws rtmath::debug::xMissingFile if the hashed shape is not found
				static boost::shared_ptr<const shapefile> loadHash(
					const Ryan_Debug::hash::HASH_t &hash);
				/// Convenience functions to load shape based on hash
				/// \throws rtmath::debug::xMissingFile if the hashed shape is not found
				static boost::shared_ptr<const shapefile> loadHash(
					const std::string &hash);

				/// When shape is partially loaded, use this routine to force a 
				/// reload from the hash database
				void loadHashLocal();
				void loadHashLocal(const Ryan_Debug::hash::HASH_t &hash);
				void loadHashLocal(const std::string &hash);

				/// Register a shape in the local hash database. Used in subsequent searches for the shape.
				void registerHash() const;
				static bool isHashStored(const std::string &hash);
				static bool isHashStored(const Ryan_Debug::hash::HASH_t &hash);
				/// Register a shape in the local hash database. Can also register a loading placeholder.
				//static void registerHash(boost::shared_ptr<shapefile>);

			};

			/// Cell information structure for convolution functions
			struct convolutionCellInfo
			{
				convolutionCellInfo();
				/// The coordinates of the point being tested
				float x, y, z;
				/// The dieelctric at the point being tested
				size_t initDiel;
				/// Kernel size operators (decimation only)
				size_t sx, sy, sz;
				/// The id of the point being tested (currently unused)
				size_t index;
				/// Return values
				size_t numFilled, numTotal;
			};
		}
	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile::shapefile &ob);
//std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);

