#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4244 ) // annoying double to float issues in boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <limits>
#include <Eigen/Dense>

#include <Ryan_Debug/debug.h>
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/common_templates.h"
#include "../rtmath/config.h"
#include "../rtmath/hash.h"
#include "../rtmath/Serialization/Serialization.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"
#include "shapestats_private.h"

namespace rtmath {
	namespace ddscat {
		namespace stats {
			const int shapeFileStatsBase::_maxVersion = 5;
			SHARED_PRIVATE bool autoHashShapes = false;
			SHARED_PRIVATE bool autoHashStats = false;
			SHARED_PRIVATE std::vector<boost::tuple<double, double, double> > defaultRots;
			SHARED_PRIVATE bool doVoronoi = true;
			SHARED_PRIVATE bool disableVoronoi = false;
			SHARED_PRIVATE bool forceRecalcStats = false;
			SHARED_PRIVATE bool prohibitStats = false;

			shapeFileStats::shapeFileStats(const shapefile::shapefile &shp)
			{
				_init();
				_shp = boost::shared_ptr<shapefile::shapefile>(new shapefile::shapefile(shp));
				calcStatsBase();
			}

			shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile::shapefile> &shp)
			{
				_init();
				// Should the shape be spliced, or not?
				_shp = shp;
				//_shp = boost::shared_ptr<shapefile::shapefile>(new shapefile::shapefile(*shp));
				calcStatsBase();
			}

			shapeFileStats::shapeFileStats() { _init();  }

			/*
			boost::shared_ptr<shapeFileStats> shapeFileStats::generate()
			{
				boost::shared_ptr<shapeFileStats> res(new shapeFileStats);
				return res;
			}
			*/

			shapeFileStatsBase::shapeFileStatsBase()
			{
				beta = 0;
				theta = 0;
				phi = 0;
				V_cell_const = 0;
				V_dipoles_const = 0;
				aeff_dipoles_const = 0;
				max_distance = 0;

				_currVersion = _maxVersion;

				// Need to fill with something for serialization to work with
				//_shp = boost::shared_ptr<shapefile>(new shapefile);
			}

			shapeFileStatsBase::~shapeFileStatsBase() { }

			shapeFileStatsBase::volumetric::volumetric()
				: V(-1), aeff_V(-1), SA(-1), aeff_SA(-1), f(-1) {} //, f_V(f), f_SA(-1) {}

			void shapeFileStatsBase::volumetric::calc(const shapeFileStatsBase *s,
				std::function<std::pair<float, float>()> fn)
			{
				auto p = fn();
				V = p.first;
				SA = p.second;
				if (V<0) return;

				aeff_V = pow(3.0 * V / (4.0f * boost::math::constants::pi<float>()), 1.f / 3.f);
				aeff_SA = pow(SA / (4.0f * boost::math::constants::pi<float>()), 0.5);
				f = s->V_dipoles_const / V;


				auto fCheck = [](float &val)
				{
					// Chech for indeterminacy
					if (val != val) val = -1.f;
					if (fabs(val) == std::numeric_limits<float>::infinity()) val = -1.f;
				};
				fCheck(f);
				fCheck(V);
				fCheck(SA);
				fCheck(aeff_V);
				fCheck(aeff_SA);
			}

			void shapeFileStatsBase::volumetric::calc(const shapeFileStatsBase *s)
			{
				if (V<0) return;

				aeff_V = pow(3.0 * V / (4.0f * boost::math::constants::pi<float>()), 1.f / 3.f);
				aeff_SA = pow(SA / (4.0f * boost::math::constants::pi<float>()), 0.5);
				f = s->V_dipoles_const / V;


				auto fCheck = [](float &val)
				{
					// Chech for indeterminacy
					if (val != val) val = -1.f;
					if (fabs(val) == std::numeric_limits<float>::infinity()) val = -1.f;
				};
				fCheck(f);
				fCheck(V);
				fCheck(SA);
				fCheck(aeff_V);
				fCheck(aeff_SA);
			}

			void shapeFileStatsBase::calcStatsRot(const rtmath::ddscat::rotations& rot) const
			{
				std::map<boost::tuple<double, double, double>, size_t > rs;
				rot.getRots(rs);
				for (const auto &i : rs)
				{
					calcStatsRot(i.first.get<0>(), i.first.get<1>(), i.first.get<2>());
				}
			}

			/// \todo Actually do minimization routine here
			shapeFileStatsBase::rotPtr shapeFileStatsBase::calcOriMinPE() const
			{
				/*
				if (_rotMinPE) return _rotMinPE;
				/// \todo Implement a ceres-solver plugin.
				/// Until then, just select the lowest PE from a grid

				for (const auto &r : rotations)
				{
					if (!_rotMinPE) _rotMinPE = r;
					else {
						if (r->PE(0, 0) < _rotMinPE->PE(0, 0)) _rotMinPE = r;
					}
				}
				*/
				return _rotMinPE;
			}

			/// \todo Fix max AR calculation
			shapeFileStatsBase::rotPtr shapeFileStatsBase::calcOriMaxAR() const
			{
				/// Calculated as part of basic stats calculation (tied in with 
				/// the convex hull algorithm).
				return _rotMaxAR;
			}

			void shapeFileStats::_init()
			{
				//::rtmath::io::Serialization::implementsSerialization<
				//	shapeFileStats, shapeFileStats_IO_output_registry,
				//	shapeFileStats_IO_input_registry, shapeFileStats_serialization>::set_sname("rtmath::ddscat::stats::shapeFileStats");

				ingest_hostname = Ryan_Debug::getHostname();
				ingest_username = Ryan_Debug::getUsername();
				using namespace boost::posix_time;
				using namespace boost::gregorian;
				ptime now = second_clock::local_time();
				ingest_timestamp = to_iso_string(now);
				ingest_rtmath_version = rtmath::debug::rev();
			}

			bool shapeFileStats::needsUpgrade() const
			{
				// Standard case
				//if (this->ingest_rtmath_version < 1636 
				//	&& this->ingest_rtmath_version > 0) return true;
				if (prohibitStats) return false;
				if (forceRecalcStats) return true;
				if (this->_currVersion >= 0 && this->_currVersion < _maxVersion) return true;
				if (_currVersion < 0 && this->ingest_rtmath_version < 1636) return true;
				return false;
			}


			bool shapeFileStatsBase::load(boost::shared_ptr<const shapefile::shapefile> nshp)
			{
				_shp = nshp;

				// Load the Voronoi diagram also
				//vd = _shp->generateVoronoi(
				//	std::string("standard"), rtmath::Voronoi::VoronoiDiagram::generateStandard);

				return true;
			}

			bool shapeFileStatsBase::load()
			{
				// Return true if shape is loaded or can be loaded (and load it)
				// Return false if shape CANNOT be loaded
				if (_shp->latticePts.rows()) return true;
				try {
					boost::shared_ptr<shapefile::shapefile> nshp = boost::const_pointer_cast<shapefile::shapefile>(_shp);
					nshp->loadHashLocal();
					return true;
				} catch (...) {return false;}

				/*
				boost::shared_ptr<const shapefile::shapefile> nshp;

				// Reload initial shape file by 1) hash or 2) filename
				using boost::filesystem::path;
				using boost::filesystem::exists;
				nshp = shapefile::shapefile::loadHash(_shp->hash());
				if (nshp)
					_shp = nshp;
				else if (serialization::detect_compressed(_shp->filename)) {
					nshp = shapefile::shapefile::generate(_shp->filename);
					//nshp = boost::shared_ptr<const shapefile::shapefile>(new shapefile::shapefile(_shp->filename));
					_shp = nshp;
				}
				else
					return false;

				// Load the Voronoi diagram also
				//vd = _shp->generateVoronoi(
				//	std::string("standard"), rtmath::Voronoi::VoronoiDiagram::generateStandard);

				return true;
				*/
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
				const std::string &shpfile, const std::string &statsfile)
			{
				using boost::filesystem::path;
				using boost::filesystem::exists;
				// When generating stats, check if the shape and stats files should be 
				// automatically stored in the hash directory
				
				// Preferentially use the local file, if it exists (the do nothing case)
				if (serialization::detect_compressed(statsfile))
				{
					boost::shared_ptr<shapeFileStats> res(new shapeFileStats); // Object creation
					res->read(statsfile);
					return res;
				}

				// Local file does not exist. Does it exist in the hash database?
				// Generate basic stats for a file.
				auto shp = rtmath::ddscat::shapefile::shapefile::generate(shpfile);
				//rtmath::ddscat::shapefile::shapefile shp(shpfile);
				boost::shared_ptr<shapeFileStats> s = loadHash(shp->hash().string());
				if (!s) {
					if (!prohibitStats)
						s = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
					else std::cerr << "Stats not found. New calculations prohibited." << std::endl;
				}
				if (s->needsUpgrade()) s->upgrade();

				if (autoHashShapes) shp->writeToHash();
				if (autoHashStats && s) s->writeToHash();

				if (statsfile.size() && s) s->writeFile(statsfile);
				return s;
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
				const boost::shared_ptr<const shapefile::shapefile> &shp)
			{
				using boost::filesystem::path;
				using boost::filesystem::exists;

				std::cerr << "Generating (or loading) stats for hash " << shp->hash().string() << std::endl;
				auto res = shapeFileStats::loadHash(shp->hash());
				if (!res) {
					if (!prohibitStats) {
						std::cerr << "Stats for hash not found. Calculating. " << std::endl;
						res = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
					}
					else std::cerr << "Stats for hash not found. New calculations prohibited." << std::endl;
				}
				if (autoHashStats && res) res->writeToHash();

				return res;
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
				const HASH_t &hash)
			{
				using boost::filesystem::path;
				using boost::filesystem::exists;

				auto res = shapeFileStats::loadHash(hash);
				if (!res && !prohibitStats) {
					auto shp = shapefile::shapefile::loadHash(hash);
					if (!shp) RTthrow debug::xMissingHash("shapefile+stats", hash.string().c_str());
					res = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
				}
				if (res)
					if (res->needsUpgrade()) res->upgrade(); // obeys stat prohibitions
				if (autoHashStats && res) res->writeToHash();

				return res;
			}

			void shapeFileStats::upgrade()
			{
				if (!needsUpgrade()) return;
				load();

				bool baseRecalced = false;
				bool rotsRecalced = false;

				auto recalcBase = [&]() {
					if (baseRecalced) return;
					// Recalculate base stats
					calcStatsBase();
					baseRecalced = true;
				};
				auto recalcRots = [&]() {
					if (rotsRecalced) return;
					// Redo each rotation
					std::set<rotData> oldRotations = rotstats;
					rotstats.clear();
					for (auto rot : oldRotations)
					{
						const basicTable &tbl = rot.get<0>();
						calcStatsRot(tbl[rotColDefs::BETA], tbl[rotColDefs::THETA], tbl[rotColDefs::PHI]);
					}
					rotsRecalced = true;
				};

				// Some logic to keep from having to recalculate everything between version upgrades
				if (this->_currVersion < 4)
				{
					recalcBase();
					recalcRots();
					this->_currVersion = _maxVersion;
				}
				// Can just follow the upgrade chain
				if (this->_currVersion == 4)
				{
					// 4-5 added more volumetric solvers and 2d projections
					calcSrms_sphere(); // These will pick up on recalculating rot stats automatically if needed
					calcSgyration();

					this->_currVersion++;
				}
				// Future upgrades can go here.

				this->_currVersion = _maxVersion;
			}

			void shapeFileStats::add_options(
				boost::program_options::options_description &cmdline,
				boost::program_options::options_description &config,
				boost::program_options::options_description &hidden)
			{
				namespace po = boost::program_options;
				using std::string;

				// hash-shape-dir and hash-stats-dir can be found in rtmath.conf. 
				// So, using another config file is useless.
				cmdline.add_options()
					//("hash-shape-dir", po::value<string>(), "Override the hash shape directory") // static option
					//("hash-stats-dir", po::value<string>(), "Override the hash stats directory") // static option
					;

				config.add_options()
					("suppress-voronoi-calcs", "Skip some Voronoi-based calculations (for faster debugging)")
					//("disable-voronoi", "Disable all Voronoi-based calculations (for faster debugging)")
					("force-recalc-stats", "Force shape stats recalculation, ignoring the cache. Used in debugging.")
					("no-recalc-stats", "Stats will only be loaded from hashes. If not found, refuse to calculate.")
					("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations for stats") // static option
					("thetas,t", po::value<string>()->default_value("0"), "Specify theta rotations for stats") // static option
					("phis,p", po::value<string>()->default_value("0"), "Specify phi rotations for stats") // static option
					//("rotations", po::value<string>(), "Specify rotations directly, in ") // static option
					/// \todo Check for options conflict / default_value priority with shape-hash
					//("do-hash-shapes", po::value<bool>()->default_value(false), "Create shape hash links")
					//("do-hash-stats", po::value<bool>()->default_value(false), "Create shape stats")
					;
				/// \todo make do-hash-* static, with automatic shape and stat writing on read?

				hidden.add_options()
					;
			}



			void shapeFileStats::process_static_options(
				boost::program_options::variables_map &vm)
			{
				namespace po = boost::program_options;
				using std::string;
				using boost::filesystem::path;

				if (vm.count("suppress-voronoi-calcs")) doVoronoi = false;
				if (vm.count("disable-voronoi")) disableVoronoi = true;
				if (vm.count("force-recalc-stats")) forceRecalcStats = true;
				if (vm.count("no-recalc-stats")) prohibitStats = true;

				//initPaths();
				//if (vm.count("hash-shape-dir")) pHashShapes = path(vm["hash-shape-dir"].as<string>());
				//if (vm.count("hash-stats-dir")) pHashStats = path(vm["hash-stats-dir"].as<string>());

				// Rotations can be used to automatically set defaults for rotated shape stats
				// Rotations are always specified (thanks to default_value)
				string sbetas = vm["betas"].as<string>();
				paramSet<double> betas(sbetas);
				string sthetas = vm["thetas"].as<string>();
				paramSet<double> thetas(sthetas);
				string sphis = vm["phis"].as<string>();
				paramSet<double> phis(sphis);

				defaultRots.clear();
				for (auto beta : betas) for (auto theta : thetas) for (auto phi : phis)
					defaultRots.push_back(boost::tuple<double, double, double>(beta, theta, phi));

				// Validate paths
				/*
				auto validateDir = [&](path p) -> bool
				{
					while (is_symlink(p))
						p = boost::filesystem::absolute(read_symlink(p), p.parent_path());
					if (!exists(p)) return false;
					if (is_directory(p)) return true;
					return false;
				};
				*/
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::loadHash(
				const HASH_t &hash)
			{
				return loadHash(boost::lexical_cast<std::string>(hash.lower));
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::loadHash(
				const std::string &hash)
			{
				boost::shared_ptr<shapeFileStats> res;

				using boost::filesystem::path;
				using boost::filesystem::exists;

				std::shared_ptr<registry::IOhandler> sh;
				std::shared_ptr<registry::IO_options> opts; // No need to set here - it gets reset by findHashObj

				if (hashStore::findHashObj(hash, "stats-r2.hdf5", sh, opts))
				{
					// However, value "hash" or "key" must be set here
					opts->setVal<std::string>("hash", hash);
					res = boost::shared_ptr<shapeFileStats>(new shapeFileStats);
					res->readMulti(sh, opts);
				}

				return res;
			}

			//boost::shared_ptr<shapeFileStats> 
			/*
			void shapeFileStats::read(const std::string &filename)
			{
				Ryan_Serialization::read<shapeFileStats>(*this, filename, "rtmath::ddscat::stats::shapeFileStats");
			}

			void shapeFileStats::write(const std::string &filename, const std::string &outtype) const
			{
				using namespace Ryan_Serialization;
				using namespace std;
				using boost::filesystem::path;
				string cmeth, uncompressed;

				string type = outtype;
				::rtmath::registry::IO_class_registry_writer<shapeFileStats>::io_multi_type dllsaver = nullptr;

				Ryan_Serialization::uncompressed_name(filename, uncompressed, cmeth);
				path pext = path(uncompressed).extension();

				// Process dll hooks first
				auto hooks = usesDLLregistry<shapeFileStats_IO_output_registry,
					::rtmath::registry::IO_class_registry_writer<shapeFileStats> >::getHooks();
				auto opts = registry::IO_options::generate();
				opts->filename(uncompressed);
				opts->filetype(type);
				for (const auto &hook : *hooks)
				{
					if (hook.io_multi_matches(nullptr,opts))
					{
						dllsaver = hook.io_multi_processor;
						if (!type.size())
							type = "dll";
						break;
					}
				}
				if (!type.size())
				{
					if (Ryan_Serialization::known_format(uncompressed)) type = "serialized";
					// Default is to write a standard shapefile
					else type = "shp";
				}


				if (type == "serialized")
				{
					Ryan_Serialization::write<shapeFileStats>(*this, filename, "rtmath::ddscat::stats::shapeFileStats");
				}
				else if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					dllsaver(nullptr, opts, this);
				}
				else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(filename.c_str());
				}
			}

			std::shared_ptr<registry::IOhandler> shapeFileStats::writeMulti(
				std::shared_ptr<rtmath::registry::IOhandler> handle,
				std::shared_ptr<rtmath::registry::IO_options> opts) const
			{
				// All of these objects can handle their own compression
				::rtmath::registry::IO_class_registry_writer<shapeFileStats>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = usesDLLregistry<shapeFileStats_IO_output_registry,
					::rtmath::registry::IO_class_registry_writer<shapeFileStats> >::getHooks();
				
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					if (hook.io_multi_matches(handle, opts))
					{
						dllsaver = hook.io_multi_processor;
						break;
					}
				}
				if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					return dllsaver(handle, opts, this);
					//return dllsaver(handle, filename, this, key, accessType);
				}
				else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(opts->filename().c_str());
				}
				return nullptr; // Should never be reached
			}
            */

			void shapeFileStats::writeToHash() const
			{
				using boost::filesystem::path;

				std::shared_ptr<registry::IOhandler> sh;
				std::shared_ptr<registry::IO_options> opts;

				// Only store hash if a storage mechanism can be found
				if (hashStore::storeHash(_shp->_localhash.string(), "stats-r2.hdf5", sh, opts))
				{
					if (!serialization::detect_compressed(opts->filename()))
						this->writeMulti(sh, opts);
				}
				else {
					std::cerr << "Cannot write shape to hash " << _shp->_localhash.string() << std::endl;
				}
			}


#define _tostr(a) #a
#define tostr(a) _tostr(a)
#define check(a) if (val == a) return std::string( tostr(a) );
			std::string rotColDefs::stringifyBasic(int val)
			{
				check(BETA); check(THETA); check(PHI); check(VERSION);
				return std::string("");
			}

			std::string rotColDefs::stringifyVector(int val)
			{
				check(MIN); check(MAX); check(SUM);
				check(SKEWNESS); check(KURTOSIS); check(MOM1);
				check(MOM2); check(PE); check(AREA_CONVEX);
				check(PERIMETER_CONVEX); check(ABS_MIN); check(ABS_MAX);
				check(ABS_MEAN); check(RMS_MEAN);
				return std::string("");
			}

			std::string rotColDefs::stringifyMatrix(int val)
			{
				check(AS_ABS); check(AS_ABS_MEAN); check(AS_RMS);
				check(MOMINERT); check(COVARIANCE);
				return std::string("");
			}

#undef _tostr
#undef tostr
#undef check
		}
	}
}

