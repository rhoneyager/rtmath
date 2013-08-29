#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/filesystem.hpp>
#include <boost/math/special_functions/round.hpp>
#include <cmath>
#include <ios>
#include <iomanip>
#include <Ryan_Serialization/serialization.h>

#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddOutputGenerator.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/hash.h"
#include "../rtmath/config.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/units.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace {
	boost::filesystem::path pHashRuns;
	//bool autoHashRuns = false;
}

namespace rtmath {
	namespace ddscat {
		ddOutput::ddOutput() : 
			freq(0), aeff(0)
		{
		}

		void ddOutput::updateAVG()
		{
			if (!scas.size()) return; // Nothing to do

			RTthrow debug::xUnimplementedFunction();

			boost::shared_ptr<ddOutputSingle> navg(new ddOutputSingle);
			//if (!avg) navg = boost::shared_ptr<ddOutputSingle>(new ddOutputSingle);
			//else navg = 

			/// \todo Adjust ddOutputSingle with a 'reload' function, which updates 
			// values of _beta and others.
			/// \todo Implement header creation here

			// Construct a ddOutputSingle based on the sca files and the weights.
			ddOutputSingle::statTableType stats;
			stats.resize(NUM_STAT_ENTRIES, 0);
			
			for (const auto &wtm : weights)
			{
				// Update the stats table
				ddOutputSingle::statTableType istats;
				wtm.first->getStatTable(istats);

				for (size_t i=0; i<NUM_STAT_ENTRIES; ++i)
					stats[i] += istats[i] * wtm.second;

				// Update the Mueller matrix table
				// Use the angle spacings of the first sca file. If the others do 
				// not fall in line, then throw an error for now.
				/// \todo Eventually just interpolate.

			}

			// Store the stats and the Mueller matrix results
		}

		void ddOutput::loadShape() const
		{
			// Load stats and shape based on the hash
			stats = shapeFileStats::loadHash(this->shapeHash);
			stats->load();
			shape = stats->_shp;
		}

		void ddOutput::writeFile(const std::string &filename) const
		{
			using namespace Ryan_Serialization;
			std::string cmeth, uncompressed;

			uncompressed_name(filename, uncompressed, cmeth);
			boost::filesystem::path p(uncompressed);
			boost::filesystem::path pext = p.extension(); // Uncompressed extension

			std::string utype = pext.string();

			// Serialization gets its own override
			if (Ryan_Serialization::known_format(utype))
			{
				Ryan_Serialization::write<ddOutput>(*this, filename, "rtmath::ddscat::ddOutput");
			} else {
				RTthrow rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		void ddOutput::readFile(const std::string &filename)
		{
			// First, detect if the file is compressed.
			using namespace Ryan_Serialization;
			std::string cmeth, target, uncompressed;
			// Combination of detection of compressed file, file type and existence.
			if (!detect_compressed(filename, cmeth, target))
				RTthrow rtmath::debug::xMissingFile(filename.c_str());
			uncompressed_name(target, uncompressed, cmeth);

			boost::filesystem::path p(uncompressed);
			boost::filesystem::path pext = p.extension(); // Uncompressed extension

			// Serialization gets its own override
			if (Ryan_Serialization::known_format(pext))
			{
				// This is a serialized file. Verify that it has the correct identifier, and 
				// load the serialized object directly
				Ryan_Serialization::read<ddOutput>(*this, filename, "rtmath::ddscat::ddOutput");
			} else {
				RTthrow rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		boost::shared_ptr<ddOutput> ddOutput::load(const std::string &filename)
		{
			boost::shared_ptr<ddOutput> res(new ddOutput);
			res->readFile(filename);
			return res;
		}

		boost::shared_ptr<ddOutput> ddOutput::generate(const std::string &dir)
		{
			// Handling typical case with only one .avg output (one frequency, one aeff)
			boost::shared_ptr<ddOutput> res(new ddOutput());
			using namespace boost::filesystem;
			using std::vector;
			path pBase(dir);
			if (!exists(pBase)) RTthrow debug::xMissingFile(dir.c_str());

			// Single level iteration through the path tree
			vector<path> cands;
			copy(directory_iterator(pBase), directory_iterator(), back_inserter(cands));
			for (const path& p : cands)
			{
				// Handle compressed files (in case my or Liu's scripts compressed the input)
				std::string uncompressed, meth;
				Ryan_Serialization::uncompressed_name(p.string(), uncompressed, meth);
				path praw(uncompressed);
				// Extract entension of files in ._ form
				// Note: some files (like mtable) have no extension. I don't use these.
				if (!praw.has_extension()) continue;
				path pext = praw.extension();
				// .avg, .sca and .fml are ddOutputSingle objects. Place them in appropriate places.
				// .out (target.out) and shape.dat refer to the shapefile. Only one needs to be loaded.
				// .par refers to the ddscat.par file. Useful in describing run inputs.
				// .log indicates a saved ddscat log file (implemented in my scripts). Useful for 
				// detecting the overall run time, but not necessary for now.
				if (pext.string() == ".avg" || pext.string() == ".sca" || pext.string() == ".fml")
				{
					boost::shared_ptr<ddOutputSingle> dds(new ddOutputSingle(p.string()));
					if (pext.string() == ".avg")
					{
						if (res->avg) RTthrow debug::xBadInput("Simple ddOutput generator accepts only one avg file");
						res->avg_original = boost::shared_ptr<ddOutputSingle>(new ddOutputSingle(*dds));
						res->avg = dds;
					}
					if (pext.string() == ".sca")
					{
						res->scas_original.insert(boost::shared_ptr<ddOutputSingle>(new ddOutputSingle(*dds)));
						res->scas.insert(dds);
					}
					if (pext.string() == ".fml")
					{
						res->fmls.insert(dds);
					}
				} else if (pext.string() == ".par")
				{
					res->parfile = boost::shared_ptr<ddPar>(new ddPar(p.string()));
				} else if (pext.string() == ".dat" || pext.string() == ".out")
				{
					if (res->shape) continue; // Only needs to be loaded once
					// Note: the hashed object is the fundamental thing here that needs to be loaded
					// The other stuff is only loaded for processing, and is not serialized directly.
					res->shape = boost::shared_ptr<shapefile>(new shapefile(p.string()));
					// Get the hash and load the stats
					res->shapeHash = res->shape->hash();
					res->stats = shapeFileStats::genStats(res->shape);
				}
			}

			// Set a basic source descriptor
			res->sources.insert(dir);
			// Set a basic tag based on the avg file's TARGET line
			{
				std::string starget;
				res->avg->getTARGET(starget);
				res->tags.insert(starget);
				// Extract the ddscat version from the target field
				// Fine "ddscat/" and read until the next space
				size_t loc = starget.find("ddscat/");
				if (loc != std::string::npos)
				{
					loc += 7;
					size_t end = 0;
					end = starget.find_first_of(' ',loc);
					if (end == std::string::npos)
						res->ddvertag = starget.substr(loc);
					else
						res->ddvertag = starget.substr(loc,end-loc);
				}
			}

			// Set the frequency and effective radius
			res->freq = units::conv_spec("um","GHz").convert(res->avg->wave());
			res->aeff = res->avg->aeff();

			// Set generator to null generator
			// Nothing need be done

			// Set weights to ddscat standards.
			/// \todo This duplicates the ensemble generator! Combine the code, using this one 
			/// as the reference implementation (sans the sca-fml recalculation)
			// Also process any fml files and regenerate full P matrices from the F matrix.
			{
				rtmath::ddscat::rotations rots;
				res->parfile->getRots(rots);
				ddWeightsDDSCAT wts(rots);
				
				float numScas = (float) res->scas.size();
				for (auto &sca : res->scas)
				{
					res->weights.insert(std::pair<boost::shared_ptr<ddOutputSingle>, float>
						(sca, (float) wts.getWeight(sca->beta(), sca->theta(), sca->phi()) ));

					// Search for corresponding fml file
					// TODO: improve this search
					auto it = std::find_if(res->fmls.cbegin(), res->fmls.cend(), 
						[&](const boost::shared_ptr<ddOutputSingle> &of)
					{
						auto perr = [](double a, double b) -> bool
						{
							if (abs((a-b)/b) < 0.0001) return true;
							if (abs(a - b) < 0.0001) return true; // Near-zero value
							return false;
						};
						if (!perr(of->beta(), sca->beta())) return false;
						if (!perr(of->theta(), sca->theta())) return false;
						if (!perr(of->phi(), sca->phi())) return false;
						return true;
					});

					// Able to fill in the remaining Mueller matrix elements
					if (it != res->fmls.cend())
					{
						ddOutputSingle::scattMatricesContainer 
							&fs = (*it)->getScattMatrices();
						ddOutputSingle::scattMatricesContainer 
							&ss = sca->getScattMatrices();
						ss.clear();
						for (auto f : fs)
						{
							boost::shared_ptr<ddScattMatrixP> np(new ddScattMatrixP(
								f->freq(), f->theta(), f->phi(), f->thetan(), f->phin()));
							np->setP(f->mueller());
							np->pol(f->pol());
							ss.insert(np);
						}
					}
				}

			}

			// Populate ms
			/// \todo Allow for multiple refractive indices
			res->ms.push_back(res->avg->getM());

			// Have the stats add in all relevant rotations
			for (auto &sca : res->scas)
			{
				res->stats->calcStatsRot(sca->beta(), sca->theta(), sca->phi());
			}

			// Save the shape in the hash location, if necessary
			res->shape->writeToHash();
			// Resave the stats in the hash location
			res->stats->writeToHash();

			return res;
		}

		boost::shared_ptr<ddOutput> ddOutput::generate(
			boost::shared_ptr<ddOutputSingle> avg,
			boost::shared_ptr<ddPar> par, 
			boost::shared_ptr<shapefile> shape)
		{
			boost::shared_ptr<ddOutput> res(new ddOutput());
			using namespace boost::filesystem;
			using std::vector;

			res->avg = avg;
			res->avg_original = boost::shared_ptr<ddOutputSingle>(new ddOutputSingle(*avg));
			res->parfile = par;
			res->shape = shape;
			res->shapeHash = shape->hash();


			// Set a basic source descriptor
			//res->sources.insert(dir);

			// Set a basic tag based on the avg file's TARGET line
			{
				std::string starget;
				res->avg->getTARGET(starget);
				res->tags.insert(starget);
				// Extract the ddscat version from the target field
				// Fine "ddscat/" and read until the next space
				size_t loc = starget.find("ddscat/");
				if (loc != std::string::npos)
				{
					loc += 7;
					size_t end = 0;
					end = starget.find_first_of(' ',loc);
					if (end == std::string::npos)
						res->ddvertag = starget.substr(loc);
					else
						res->ddvertag = starget.substr(loc,end-loc);
				}
			}

			// Set the frequency and effective radius
			res->freq = units::conv_spec("um","GHz").convert(res->avg->wave());
			res->aeff = res->avg->aeff();

			// Set generator to null generator
			// Nothing need be done


			// Populate ms
			/// \todo Allow for multiple refractive indices
			res->ms.push_back(res->avg->getM());

			// Save the shape in the hash location, if necessary
			res->shape->writeToHash();
			// Resave the stats in the hash location
			res->stats->writeToHash();

			return res;
		}

		void ddOutput::expand(const std::string &outdir, bool writeShape) const
		{
			using namespace boost::filesystem;
			path pOut(outdir);
			if (exists(pOut))
			{
				path pSym = debug::expandSymlink(pOut);
				if (!is_directory(pSym))
					RTthrow debug::xPathExistsWrongType(outdir.c_str());
			} else {
				create_directory(pOut);
			}

			// Write par file
			this->parfile->writeFile( (path(pOut)/"ddscat.par").string());

			// Write shape file
			if (writeShape)
			{
				if (!this->shape) this->loadShape();
				shape->write( (path(pOut)/"target.out").string() );
			}

			// Write avg file
			avg->writeFile( (path(pOut)/"w000r000.avg").string() );

			auto oname = [](const boost::filesystem::path &base,
				boost::shared_ptr<ddOutputSingle> d) -> std::string
			{
				std::ostringstream n;
				/// \todo add a unique orientation hash
				n << d->beta() << "-" << d->theta() << 
					"-" << d->phi();
				return (base/path(n.str())).string();
			};
			auto onameb = [](const boost::filesystem::path &base,
				size_t i, size_t ni) -> std::string
			{
				using namespace std;
				ostringstream n;
				n << "w000r000k";
				n.width(ni);
				n.fill('0');
				n << i;
				return (base/path(n.str())).string();
			};

			// Write fmls
			size_t i=0;
			for (const auto &fml : fmls)
			{
				//std::string fname = oname(pOut,fml);
				std::string fname = onameb(pOut,i, fmls.size());
				fname.append(".fml");
				fml->writeFile(fname);
			}

			// Write original scas
			i=0;
			for (const auto &sca : scas_original)
			{
				//std::string fname = oname(pOut,fml);
				std::string fname = onameb(pOut,i, scas_original.size());
				fname.append(".sca-original");
				sca->writeFile(fname, ".sca");
			}

			// Write scas
			i=0;
			for (const auto &sca : scas)
			{
				//std::string fname = oname(pOut,fml);
				std::string fname = onameb(pOut,i, scas.size());
				fname.append(".sca");
				sca->writeFile(fname);
			}

			// Write out the weighting table
			{
				std::ofstream owt( (path(pOut)/"weights.tsv").string() );
				owt << "Theta\tPhi\tBeta\tWeight\n";
				for (auto w : weights)
				{
					owt << w.first->theta() << "\t" << w.first->phi() << "\t"
						<< w.first->beta() << "\t" << w.second << "\n";
				}
			}
		}

		void ddOutput::add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden)
		{
			namespace po = boost::program_options;
			using std::string;

			// hash-shape-dir and hash-stats-dir can be found in rtmath.conf. 
			// So, using another config file is useless.
			cmdline.add_options()
				("hash-runs-dir", po::value<string>(), "Override the hash runs directory") // static option
				;

			config.add_options()
				;

			hidden.add_options()
				;
		}

		void ddOutput::process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;
			using boost::filesystem::path;

			initPaths();
			if (vm.count("hash-runs-dir")) pHashRuns = path(vm["hash-runs-dir"].as<string>());

			// Validate paths
			auto validateDir = [&](path p) -> bool
			{
				while (is_symlink(p))
					p = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				if (!exists(p)) return false;
				if (is_directory(p)) return true;
				return false;
			};
			if (!validateDir(pHashRuns)) RTthrow debug::xMissingFile(pHashRuns.string().c_str());
		}

		void ddOutput::initPaths()
		{
			using namespace boost::filesystem;
			using std::string;
			if (!pHashRuns.empty()) return;

			auto conf = rtmath::config::loadRtconfRoot();
			string runsDir;
			auto chash = conf->getChild("ddscat")->getChild("hash");
			chash->getVal<string>("runsDir",runsDir);
			
			pHashRuns = path(runsDir);
		}

		void ddOutput::getHashPaths(boost::filesystem::path &pHashRunsO)
		{
			pHashRunsO = pHashRuns;
		}

		void ddOutput::writeToHash() const
		{
			using boost::filesystem::path;

			path pHashRunsO;
			getHashPaths(pHashRunsO);

			path pHashRun = storeHash(pHashRunsO, shapeHash);
			// Append the name to the hash
			std::string n = genName();
			pHashRun = pHashRun.parent_path() / path(n);
			if (!Ryan_Serialization::detect_compressed(pHashRun.string()))
				writeFile(pHashRun.string());
		}

		std::string ddOutput::genName() const
		{
			std::string res;

			/* Name follows this pattern:
			 - shape hash
			 - frequency (to nearest tenth)
			 - aeff (to nearest tenth)
			 - number of sca matrices
			 - rotations
			 - ddscat version tag
			 */

			boost::filesystem::path p;
			rotations rots;
			parfile->getRots(rots);

			std::ostringstream out;
			out << shapeHash.lower << "-"
				<< (boost::math::round((float) freq*10.f)/10.f) << "-"
				<< (boost::math::round((float) aeff*10.f)/10.f) << "-"
				<< scas.size() << "-"
				<< rots.bN() << "-" << rots.tN() << "-" << rots.pN() << "-"
				<< ddvertag
				<< ".xml";

			res = out.str();
			return res;
		}

		/*
		void ddOutput::clear()
		{
		_init();
		_outputSingleRaw.clear();
		_mapOutputSingleRaw.clear();
		}

		void ddOutput::insert(const std::shared_ptr<const ddscat::ddOutputSingle> &obj)
		{
		_outputSingleRaw.insert(obj);
		if (_mapOutputSingleRaw.count(obj->genCoords()) == 0)
		_mapOutputSingleRaw[obj->genCoords()] = obj;
		}

		void ddOutput::get(const coords::cyclic<double> &crds, 
		std::shared_ptr<const ddscat::ddOutputSingle> &obj,
		bool interpolate) const
		{
		if (_mapOutputSingleRaw.count(crds))
		{
		obj = _mapOutputSingleRaw.at(crds);
		return;
		}

		if (interpolate)
		{
		obj = nullptr;
		throw rtmath::debug::xUnimplementedFunction();
		return;
		}

		// Failure to get appropriate output
		obj = nullptr;
		// TODO: throw stuff?
		//throw rtmath::debug::xUnimplementedFunction();
		return;
		}

		void ddOutput::ensemble(const ddOutputEnsemble &provider, ddOutputSingle &res) const
		{
		// Will pass _mapOutputSingleRaw to the provider,
		// as it already has the coordinate mappings.
		// The ddOutputSingle entry will not refer to the originating ScattMatrices,
		// as this many references would be too complex. Instead, it will be standalone.
		provider.genEnsemble(_mapOutputSingleRaw,res);
		}
		*/
	}
}
