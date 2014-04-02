#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4244 ) // annoying double to float issues in boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>
#include <limits>
#include <Eigen/Dense>

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/common_templates.h"
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"
#include "shapestats_private.h"

namespace rtmath {
	namespace ddscat {
		namespace stats {
			const unsigned int shapeFileStatsBase::_maxVersion = SHAPESTATS_VERSION;
			SHARED_PRIVATE boost::filesystem::path pHashShapes, pHashStats;
			SHARED_PRIVATE bool autoHashShapes = false;
			SHARED_PRIVATE bool autoHashStats = false;
			SHARED_PRIVATE std::vector<boost::tuple<double, double, double> > defaultRots;
			SHARED_PRIVATE bool doVoronoi = true;
			SHARED_PRIVATE bool disableVoronoi = false;
			SHARED_PRIVATE bool forceRecalcStats = false;


			bool rotComp::operator()(const boost::shared_ptr<const shapeFileStatsRotated> &lhs,
				const boost::shared_ptr<const shapeFileStatsRotated> &rhs) const
			{
				if (lhs->beta != rhs->beta) return lhs->beta < rhs->beta;
				if (lhs->theta != rhs->theta) return lhs->theta < rhs->theta;
				if (lhs->phi != rhs->phi) return lhs->phi < rhs->phi;
				return false;
			}

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


			shapeFileStatsBase::shapeFileStatsBase()
			{
				beta = 0;
				theta = 0;
				phi = 0;
				V_cell_const = 0;
				V_dipoles_const = 0;
				aeff_dipoles_const = 0;
				max_distance = 0;

				_currVersion = -1;

				// Need to fill with something for serialization to work with
				//_shp = boost::shared_ptr<shapefile>(new shapefile);
			}

			shapeFileStatsBase::~shapeFileStatsBase() { }


			void shapeFileStatsBase::volumetric::calc(const shapeFileStatsBase *s,
				std::function<std::pair<float, float>()> fn)
			{
				auto p = fn();
				V = p.first;
				SA = p.second;
				if (V<0) return;

				aeff_V = pow(3.0 * V / (4.0f * boost::math::constants::pi<float>()), 1.f / 3.f);
				aeff_SA = pow(SA / (4.0f * boost::math::constants::pi<float>()), 0.5);
				f = V / s->V_cell_const;


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
				f = s->V_cell_const / V;


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

			shapeFileStatsBase::rotPtr shapeFileStatsBase::calcOriMinPE() const
			{
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

				return _rotMinPE;
			}

			shapeFileStatsBase::rotPtr shapeFileStatsBase::calcOriMaxAR() const
			{
				/// Calculated as part of basic stats calculation (tied in with 
				/// the convex hull algorithm).
				return _rotMaxAR;
			}

			void shapeFileStats::_init()
			{
				::rtmath::io::Serialization::implementsSerialization<
					shapeFileStats, shapeFileStats_IO_output_registry,
					shapeFileStats_IO_input_registry, shapeFileStats_serialization>::set_sname("rtmath::ddscat::stats::shapeFileStats");
			}

			bool shapeFileStats::needsUpgrade() const
			{
				// Standard case
				if (this->_currVersion >= 0 && this->_currVersion < _maxVersion) return true;
				return false;
			}

			bool shapeFileStatsBase::load()
			{
				// Return true if shape is loaded or can be loaded (and load it)
				// Return false if shape CANNOT be loaded
				if (_shp->latticePts.rows()) return true;

				boost::shared_ptr<shapefile::shapefile> nshp;

				// Reload initial stats file by 1) hash or 2) filename
				using boost::filesystem::path;
				using boost::filesystem::exists;
				std::vector<std::string> extensions;
				extensions.push_back(".xml");
				extensions.push_back(".st");
				path pHashShape = findHash(pHashShapes, _shp->hash(), extensions);
				if (!pHashShape.empty())
					nshp = boost::shared_ptr<shapefile::shapefile>(new shapefile::shapefile(pHashShape.string()));
				//path pHashShape = pHashShapes / boost::lexical_cast<std::string>(_shp->hash().lower);
				//if (Ryan_Serialization::detect_compressed(pHashShape.string()))
				//	nshp = boost::shared_ptr<shapefile>(new shapefile(pHashShape.string()));
				//else if (boost::filesystem::exists(boost::filesystem::path(_shp->filename)))
				else if (Ryan_Serialization::detect_compressed(_shp->filename))
					nshp = boost::shared_ptr<shapefile::shapefile>(new shapefile::shapefile(_shp->filename));
				else
					return false;
				_shp = nshp;
				return true;
			}

			void shapeFileStats::getHashPaths(
				boost::filesystem::path &HashShapes,
				boost::filesystem::path &HashStats)
			{
				initPaths();
				HashShapes = pHashShapes;
				HashStats = pHashStats;
			}


			boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
				const std::string &shpfile, const std::string &statsfile)
			{
				using boost::filesystem::path;
				using boost::filesystem::exists;

				boost::shared_ptr<shapeFileStats> res(new shapeFileStats); // Object creation

				// Preferentially use the local file, if it exists
				if (Ryan_Serialization::detect_compressed(statsfile))
				{
					res->read(statsfile);
					return res;
				}
				/*
				{
				::Ryan_Serialization::read<shapeFileStats>(*res, statsfile, "rtmath::ddscat::shapeFileStats");
				return res;
				}
				*/

				// Local file does not exist. Does it exist in the hash database?
				// Generate basic stats for a file.
				rtmath::ddscat::shapefile::shapefile shp(shpfile);

				// Check the hash to see if it's already been done before
				// Also see if the statsfile exists
				using namespace boost::filesystem;

				path pHashShape = storeHash(pHashShapes, shp.hash());
				//pHashShapes / boost::lexical_cast<std::string>(shp.hash().lower);
				if (!Ryan_Serialization::detect_compressed(pHashShape.string()) && autoHashShapes)
				{
                    /// \todo Somehow, automatically write compression
					shp.write(pHashShape.string());
				}
				path pHashStat = storeHash(pHashStats, shp.hash()); // Path + hash. No extension yet.
				// Query the directory for all files matching the pHashStat at the beginning.
				using std::vector;
				vector<path> cands;
				copy(directory_iterator(pHashStat.parent_path()), directory_iterator(), back_inserter(cands));
				// There really should only be one file matching a hash in a directory. Pick the first.
				decltype(cands.begin()) it = std::find_if(cands.begin(), cands.end(), [&pHashStat](const path &p)
				{
					// Left-sided compare
					if (p.filename().string().find(pHashStat.filename().string()) == 0) return true;
					return false;
				});
				if (it != cands.end() && !forceRecalcStats)
				{
					::Ryan_Serialization::read<shapeFileStats>(*res, it->string(), "rtmath::ddscat::stats::shapeFileStats");
				}
				else {
					// This takes care or base stat calculation and rotation stat calculations
					res = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
				}

				if (statsfile.size())
				{
					::Ryan_Serialization::write<rtmath::ddscat::stats::shapeFileStats >(*res, statsfile, "rtmath::ddscat::stats::shapeFileStats");
				}
				if (autoHashStats)
				{
					// Query Ryan_Serialization for the default extension.
					path pRes;
					Ryan_Serialization::serialization_method sm = Ryan_Serialization::select_format(pHashStat, pRes);
					::Ryan_Serialization::write<rtmath::ddscat::stats::shapeFileStats >(*res, pRes.string(), "rtmath::ddscat::stats::shapeFileStats");
				}

				return res;
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
				const boost::shared_ptr<const shapefile::shapefile> &shp)
			{
				using boost::filesystem::path;
				using boost::filesystem::exists;

				boost::shared_ptr<shapeFileStats> res(new shapeFileStats); // Object creation

				// Check the hash to see if it's already been done before
				// Also see if the statsfile exists
				using boost::filesystem::path;
				using boost::filesystem::exists;

				initPaths();
				path pHashShape = storeHash(pHashShapes, shp->hash());
				if (!Ryan_Serialization::detect_compressed(pHashShape.string()) && autoHashShapes)
				{
                    /// \todo Reenable compression (may involve a writeMulti call with options)
					shp->write(pHashShape.string());
				}
				path pHashStat = storeHash(pHashStats, shp->hash());
				//pHashStats / boost::lexical_cast<std::string>(shp.hash().lower);
				if (Ryan_Serialization::detect_compressed(pHashStat.string()) && !forceRecalcStats)
					::Ryan_Serialization::read<shapeFileStats>(*res, pHashStat.string(), "rtmath::ddscat::stats::shapeFileStats");
				else
				{
					// This takes care or base stat calculation and rotation stat calculations
					res = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
				}

				if (autoHashStats)
				{
					::Ryan_Serialization::write<rtmath::ddscat::stats::shapeFileStats >(*res, pHashStat.string(), "rtmath::ddscat::stats::shapeFileStats");
				}

				return res;
			}

			void shapeFileStats::upgrade()
			{
				if (!needsUpgrade()) return;
				load();

				// Recalculate base stats
				calcStatsBase();

				// Redo each rotation
				std::set<boost::shared_ptr<const shapeFileStatsRotated>, rotComp > oldRotations
					= rotations;
				rotations.clear();
				for (auto rot : oldRotations)
				{
					if (rot->needsUpgrade())
						calcStatsRot(rot->beta, rot->theta, rot->phi);
					else
						rotations.insert(rot);
				}

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
					("hash-shape-dir", po::value<string>(), "Override the hash shape directory") // static option
					("hash-stats-dir", po::value<string>(), "Override the hash stats directory") // static option
					;

				config.add_options()
					("suppress-voronoi-calcs", "Skip some Voronoi-based calculations (for faster debugging)")
					//("disable-voronoi", "Disable all Voronoi-based calculations (for faster debugging)")
					("force-recalc-stats", "Force shape stats recalculation, ignoring the cache. Used in debugging.")
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

				initPaths();
				if (vm.count("hash-shape-dir")) pHashShapes = path(vm["hash-shape-dir"].as<string>());
				if (vm.count("hash-stats-dir")) pHashStats = path(vm["hash-stats-dir"].as<string>());

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
				auto validateDir = [&](path p) -> bool
				{
					while (is_symlink(p))
						p = boost::filesystem::absolute(read_symlink(p), p.parent_path());
					if (!exists(p)) return false;
					if (is_directory(p)) return true;
					return false;
				};
				if (!validateDir(pHashShapes)) RTthrow debug::xMissingFile(pHashShapes.string().c_str());
				if (!validateDir(pHashStats)) RTthrow debug::xMissingFile(pHashStats.string().c_str());
			}

			void shapeFileStats::initPaths()
			{
				using namespace boost::filesystem;
				using std::string;
				if (!pHashShapes.empty()) return;

				auto conf = rtmath::config::loadRtconfRoot();
				string shapeDir;
				auto chash = conf->getChild("ddscat")->getChild("hash");
				chash->getVal<string>("shapeDir", shapeDir);
				//if (vm.count("hash-shape-dir")) shapeDir = vm["hash-shape-dir"].as<string>();
				string statsDir;
				chash->getVal<string>("statsDir", statsDir);
				//if (vm.count("hash-stats-dir")) statsDir = vm["hash-stats-dir"].as<string>();

				pHashShapes = path(shapeDir);
				pHashStats = path(statsDir);
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::loadHash(
				const HASH_t &hash)
			{
				return loadHash(boost::lexical_cast<std::string>(hash.lower));
			}

			boost::shared_ptr<shapeFileStats> shapeFileStats::loadHash(
				const std::string &hash)
			{
				boost::shared_ptr<shapeFileStats> res(new shapeFileStats);

				using boost::filesystem::path;
				using boost::filesystem::exists;

				path pHashShapes;
				path pHashStats;
				shapeFileStats::getHashPaths(pHashShapes, pHashStats);

				path pHashStat = findHash(pHashStats, hash);
				if (!pHashStat.empty())
				{
					res = boost::shared_ptr<shapeFileStats>(new shapeFileStats());
					res->read(pHashStat.string());
				}
				else
					throw rtmath::debug::xMissingFile(hash.c_str());
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

				path pHashShapes;
				path pHashStats;
				shapeFileStats::getHashPaths(pHashShapes, pHashStats);

				path pHashStat = storeHash(pHashStats, _shp->_localhash);
				write(pHashStat.string());
			}

		}
	}
}

