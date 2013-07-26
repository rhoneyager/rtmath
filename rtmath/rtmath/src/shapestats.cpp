#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4244 ) // annoying double to float issues in boost

#include <set>
#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>
#include <limits>
#include <Eigen/Dense>

#include <Ryan_Serialization/serialization.h>

//#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/common_templates.h"
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace {
	std::vector<boost::tuple<double, double, double> > defaultRots;
	boost::filesystem::path pHashShapes, pHashStats;
	bool autoHashShapes = false;
	bool autoHashStats = false;
}

namespace rtmath {
	namespace ddscat {

		const unsigned int shapeFileStatsBase::_maxVersion = SHAPESTATS_VERSION;

		bool rotComp::operator()(const boost::shared_ptr<const shapeFileStatsRotated> &lhs,
			const boost::shared_ptr<const shapeFileStatsRotated> &rhs) const
		{
			if (lhs->beta != rhs->beta) return lhs->beta < rhs->beta;
			if (lhs->theta != rhs->theta) return lhs->theta < rhs->theta;
			if (lhs->phi != rhs->phi) return lhs->phi < rhs->phi;
			return false;
		}

		shapeFileStats::shapeFileStats(const shapefile &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(shp));
			calcStatsBase();
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(*shp));
			calcStatsBase();
		}

		shapeFileStats::shapeFileStats() { }

		shapeFileStatsBase::shapeFileStatsBase()
		{
			beta = 0;
			theta = 0;
			phi = 0;
			V_cell_const = 0;
			V_dipoles_const = 0;
			aeff_dipoles_const = 0;
			max_distance = 0;

			a_circum_sphere = 0;
			V_circum_sphere = 0;
			SA_circum_sphere = 0;
			V_convex_hull = 0;
			aeff_V_convex_hull = 0;
			SA_convex_hull = 0;
			aeff_SA_convex_hull = 0;

			V_ellipsoid_max = 0;
			aeff_ellipsoid_max = 0;
			V_ellipsoid_rms = 0;
			aeff_ellipsoid_rms = 0;

			f_circum_sphere = 0;
			f_convex_hull = 0;
			f_ellipsoid_max = 0;
			f_ellipsoid_rms = 0;

			_currVersion = -1;

			// Need to fill with something for serialization to work with
			//_shp = boost::shared_ptr<shapefile>(new shapefile);
		}

		shapeFileStatsBase::~shapeFileStatsBase() { }

		void shapeFileStatsBase::calcStatsBase()
		{
			using namespace std;
			// Do calculations of the center of mass, the tensor quantities, and other stuff
			// The functions called here are all indep. of the initial state, as mass, density,
			// volume and everything else have been calculated already.

			// Define the accumulators that we want
			// For each axis, get min, max and the other statistics about the distribution

			// Iterate accumulator as function of radial distance from center of mass

			// Pull in some vars from the shapefile
			const size_t _N = _shp->numPoints;
			
			if (!_N)
			{
				throw rtmath::debug::xBadInput("Stats cannot be calculated because the shapefile is not loaded.");
			}

			// Calculate volume elements
			float dxdydz = _shp->d(0) * _shp->d(1) * _shp->d(2);
			V_cell_const = dxdydz;
			V_dipoles_const = dxdydz * _N;
			aeff_dipoles_const = pow(V_dipoles_const*3.f/(4.f*boost::math::constants::pi<float>()),1.f/3.f);

			const Eigen::Array3f &a1 = _shp->a1;
			const Eigen::Array3f &a2 = _shp->a2;
			const Eigen::Array3f &a3 = _shp->a3;
			// Figure out the base rotation from a1 = <1,0,0>, a2 = <0,1,0> that 
			// gives the current a1, a2.

			float thetar, betar, phir;
			// Theta is the angle between a1 and xlf
			// From dot product, theta = acos(a1.xlf)
			float dp = a1(0); // need only x component, as xlf is the unit vector in +x
			thetar = acos(dp);

			// From a1 = x_lf*cos(theta) + y_lf*sin(theta)*cos(phi) + z_lf*sin(theta)*sin(phi),
			// can use either y_lf or z_lf components to get phi
			float stheta = sin(thetar);
			if (thetar)
			{
				float acphi = a1(1) / stheta;
				phir = acos(acphi);

				// Finally, a2_x = -sin(theta)cos(beta)
				float cbeta = a2(0) / stheta * -1.f;
				betar = acos(cbeta);
			} else {
				// theta is zero, so gimbal locking occurs. assume phi = 0.
				phir = 0;
				// must use alternate definition to get beta
				float cosbeta = a2(1);
				betar = acos(cosbeta);
			}

			// thetar, betar, phir are in radians
			// convert to degrees
			{
				double scale = 180.0/(boost::math::constants::pi<double>());
				beta = betar * scale;
				theta = thetar * scale;
				phi = phir * scale;
			}

			rotationMatrix<float>(theta, phi, beta, rot);
			invrot = rot.inverse();

			// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
			using namespace boost::accumulators;
			//using namespace boost::accumulators::tag;
			
			// Do two passes to be able to renormalize coordinates
			accumulator_set<float, stats<tag::mean, tag::min, tag::max> > m_x, m_y, m_z;
			//for (auto it = _shp->latticePtsStd.begin(); it != _shp->latticePtsStd.end(); ++it)
			for (size_t i = 0; i < _shp->numPoints; i++)
			{
				auto it = _shp->latticePts.block<1,3>(i,0);
				m_x((it)(0));
				m_y((it)(1));
				m_z((it)(2));
			}

			b_min(0) = boost::accumulators::min(m_x);
			b_min(1) = boost::accumulators::min(m_y);
			b_min(2) = boost::accumulators::min(m_z);

			b_max(0) = boost::accumulators::max(m_x);
			b_max(1) = boost::accumulators::max(m_y);
			b_max(2) = boost::accumulators::max(m_z);

			b_mean(0) = boost::accumulators::mean(m_x);
			b_mean(1) = boost::accumulators::mean(m_y);
			b_mean(2) = boost::accumulators::mean(m_z);

			convexHull cvHull(_shp->latticePtsStd);
			cvHull.constructHull();
			max_distance = cvHull.maxDiameter();
			
			a_circum_sphere = max_distance / 2.0;
			V_circum_sphere = boost::math::constants::pi<float>() * 4.0f * pow(a_circum_sphere,3.0f) / 3.0f;
			SA_circum_sphere = boost::math::constants::pi<float>() * 4.0f * pow(a_circum_sphere,2.0f);

			V_convex_hull = cvHull.volume();
			aeff_V_convex_hull = pow(3.0 * V_convex_hull / (4.0f * boost::math::constants::pi<float>()),1.f/3.f);
			SA_convex_hull = cvHull.surfaceArea();
			aeff_SA_convex_hull = pow(SA_convex_hull / (4.0f * boost::math::constants::pi<float>()),0.5);

			_currVersion = _maxVersion;

			// Calculate rotated stats to avoid having to duplicate code
			// From the 0,0,0 rotation,
			{
				// At beginning by default, as it is the only entry at this point!
				auto pdr = calcStatsRot(0,0,0);
				V_ellipsoid_max = boost::math::constants::pi<float>() / 6.0f;
				// Using diameters, and factor in prev line reflects this

				/// \todo Check V_ellipsoid_max and V_ellipsoid_rms calculations!
				V_ellipsoid_max *= pdr->max(0,0) - pdr->min(0,0);
				V_ellipsoid_max *= pdr->max(0,1) - pdr->min(0,1);
				V_ellipsoid_max *= pdr->max(0,2) - pdr->min(0,2);

				V_ellipsoid_rms = 4.0f * boost::math::constants::pi<float>() / 3.0f;
				V_ellipsoid_rms *= pdr->max(0,0) - pdr->min(0,0);
				V_ellipsoid_rms *= pdr->max(0,1) - pdr->min(0,1);
				V_ellipsoid_rms *= pdr->max(0,2) - pdr->min(0,2);

				aeff_ellipsoid_max = pow(3.0f * V_ellipsoid_max / (4.0f * boost::math::constants::pi<float>()),1.f/3.f);
				aeff_ellipsoid_rms = pow(3.0f * V_ellipsoid_rms / (4.0f * boost::math::constants::pi<float>()),1.f/3.f);
			}

			// Volume fractions
			f_circum_sphere = V_dipoles_const / V_circum_sphere;
			f_convex_hull = V_dipoles_const / V_convex_hull;
			f_ellipsoid_max = V_dipoles_const / V_ellipsoid_max;
			f_ellipsoid_rms = V_dipoles_const / V_ellipsoid_rms;

			auto fCheck = [](float &val)
			{
				// Chech for indeterminacy
				if (val != val) val = -1.f;
				if (fabs(val) == std::numeric_limits<float>::infinity()) val = -1.f;
			};

			fCheck(f_circum_sphere);
			fCheck(f_convex_hull);
			fCheck(f_ellipsoid_max);
			fCheck(f_ellipsoid_rms);
			
			// Calculate all default (from config or command-line) rotations
			for (auto rot : defaultRots)
			{
				calcStatsRot(rot.get<0>(), rot.get<1>(), rot.get<2>());
			}
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
			if (_shp->latticePts.rows() ) return true;

			boost::shared_ptr<shapefile> nshp;

			// Reload initial stats file by 1) hash or 2) filename
			using boost::filesystem::path;
			using boost::filesystem::exists;
			path pHashShape = findHash(pHashShapes, _shp->hash());
			if (!pHashShape.empty())
				nshp = boost::shared_ptr<shapefile>(new shapefile(pHashShape.string()));
			//path pHashShape = pHashShapes / boost::lexical_cast<std::string>(_shp->hash().lower);
			//if (Ryan_Serialization::detect_compressed(pHashShape.string()))
			//	nshp = boost::shared_ptr<shapefile>(new shapefile(pHashShape.string()));
			//else if (boost::filesystem::exists(boost::filesystem::path(_shp->filename)))
			else if (Ryan_Serialization::detect_compressed(_shp->filename))
				nshp = boost::shared_ptr<shapefile>(new shapefile(_shp->filename));
			else
				return false;
			_shp = nshp;
			return true;
		}

		void shapeFileStats::getHashPaths(
			boost::filesystem::path &HashShapes,
			boost::filesystem::path &HashStats)
		{
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
			//if (exists(path(statsfile)))
			{
				::Ryan_Serialization::read<shapeFileStats>(*res, statsfile, "rtmath::ddscat::shapeFileStats");
				return res;
			}

			// Local file does not exist. Does it exist in the hash database?
			// Generate basic stats for a file.
			rtmath::ddscat::shapefile shp(shpfile);

			// Check the hash to see if it's already been done before
			// Also see if the statsfile exists
			using boost::filesystem::path;
			using boost::filesystem::exists;

			path pHashShape = storeHash(pHashShapes,shp.hash());
			//pHashShapes / boost::lexical_cast<std::string>(shp.hash().lower);
			if (!Ryan_Serialization::detect_compressed(pHashShape.string()) && autoHashShapes)
			{
				shp.write(pHashShape.string(), true);
			}
			path pHashStat = storeHash(pHashStats,shp.hash());
			//pHashStats / boost::lexical_cast<std::string>(shp.hash().lower);
			if (Ryan_Serialization::detect_compressed(pHashStat.string()))
				::Ryan_Serialization::read<shapeFileStats>(*res, pHashStat.string(), "rtmath::ddscat::shapeFileStats");
			else
			{
				// This takes care or base stat calculation and rotation stat calculations
				res = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
			}

			if (statsfile.size())
			{
				::Ryan_Serialization::write<rtmath::ddscat::shapeFileStats >(*res,statsfile,"rtmath::ddscat::shapeFileStats");
			}
			if (autoHashStats)
			{
				::Ryan_Serialization::write<rtmath::ddscat::shapeFileStats >(*res,pHashStat.string(),"rtmath::ddscat::shapeFileStats");
			}

			return res;
		}

		boost::shared_ptr<shapeFileStats> shapeFileStats::genStats(
			const boost::shared_ptr<shapefile> &shp)
		{
			using boost::filesystem::path;
			using boost::filesystem::exists;

			boost::shared_ptr<shapeFileStats> res(new shapeFileStats); // Object creation

			// Check the hash to see if it's already been done before
			// Also see if the statsfile exists
			using boost::filesystem::path;
			using boost::filesystem::exists;

			path pHashShape = storeHash(pHashShapes,shp->hash());
			if (!Ryan_Serialization::detect_compressed(pHashShape.string()) && autoHashShapes)
			{
				shp->write(pHashShape.string(), true);
			}
			path pHashStat = storeHash(pHashStats,shp->hash());
			//pHashStats / boost::lexical_cast<std::string>(shp.hash().lower);
			if (Ryan_Serialization::detect_compressed(pHashStat.string()))
				::Ryan_Serialization::read<shapeFileStats>(*res, pHashStat.string(), "rtmath::ddscat::shapeFileStats");
			else
			{
				// This takes care or base stat calculation and rotation stat calculations
				res = boost::shared_ptr<shapeFileStats>(new shapeFileStats(shp));
			}

			if (autoHashStats)
			{
				::Ryan_Serialization::write<rtmath::ddscat::shapeFileStats >(*res,pHashStat.string(),"rtmath::ddscat::shapeFileStats");
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
				defaultRots.push_back(boost::tuple<double,double,double>(beta,theta,phi));

			// Validate paths
			auto validateDir = [&](path p) -> bool
			{
				while (is_symlink(p))
					p = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				if (!exists(p)) return false;
				if (is_directory(p)) return true;
				return false;
			};
			if (!validateDir(pHashShapes)) throw debug::xMissingFile(pHashShapes.string().c_str());
			if (!validateDir(pHashStats)) throw debug::xMissingFile(pHashStats.string().c_str());
		}

		void shapeFileStats::initPaths()
		{
			using namespace boost::filesystem;
			using std::string;
			if (!pHashShapes.empty()) return;

			auto conf = rtmath::config::loadRtconfRoot();
			string shapeDir;
			auto chash = conf->getChild("ddscat")->getChild("hash");
			chash->getVal<string>("shapeDir",shapeDir);
			//if (vm.count("hash-shape-dir")) shapeDir = vm["hash-shape-dir"].as<string>();
			string statsDir;
			chash->getVal<string>("statsDir",statsDir);
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
		void shapeFileStats::read(const std::string &src)
		{
			//boost::shared_ptr<shapeFileStats> res(new shapeFileStats);
			Ryan_Serialization::read<shapeFileStats>(*this,src, "rtmath::ddscat::shapeFileStats");
			//return res;
		}

		void shapeFileStats::write(const std::string &filename) const
		{
			Ryan_Serialization::write<shapeFileStats>(*this, filename, "rtmath::ddscat::shapeFileStats");
		}

		void shapeFileStats::writeToHash() const
		{
			using boost::filesystem::path;

			path pHashShapes;
			path pHashStats;
			shapeFileStats::getHashPaths(pHashShapes, pHashStats);

			path pHashStat = storeHash(pHashStats, _shp->hash);
			write(pHashStat.string());
		}
	}
}

