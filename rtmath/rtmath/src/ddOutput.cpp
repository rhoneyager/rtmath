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
#include <thread>
#include <mutex>

#include "../rtmath/zeros.h"
#include "../rtmath/refract.h"
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOriData.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/ddUtil.h"
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
	namespace registry {
	
		template struct IO_class_registry_writer
			<::rtmath::ddscat::ddOutput>;
		
		template class usesDLLregistry<
			::rtmath::ddscat::ddOutput_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddOutput> >;
		
		template struct IO_class_registry_reader
			<::rtmath::ddscat::ddOutput>;
		
		template class usesDLLregistry<
			::rtmath::ddscat::ddOutput_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddOutput> >;
		
	}

	namespace ddscat {
		/*
		void ddOutput::doImport()
		{
			size_t nOris = static_cast<size_t>(oridata_s.rows());
			size_t nAngles = 0;
			for (size_t i = 0; i < nOris; ++i)
			{
				if (oridata_i(i, stat_entries::DOWEIGHT))
				{
					nAngles += oridata_i(im stat_entries::NUMF);
				}
			}

			resizeFML(nAngles);

			for (size_t i = 0, j = 0; i < nOris; ++i)
			{
				if (oridata_i(i, stat_entries::DOWEIGHT))
				{

				}
			}
		}
		*/

		ddOutput::ddOutput() : 
			freq(0), aeff(0), temp(0)
		{
			resize(0, 0);
		}

		/*
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
				// not fall in line, then RTthrow an error for now.
				/// \todo Eventually just interpolate.

			}

			// Store the stats and the Mueller matrix results
		}
		*/
		void ddOutput::loadShape() const
		{
			// Load stats and shape based on the hash
			stats = stats::shapeFileStats::loadHash(this->shapeHash);
			stats->load();
			shape = stats->_shp;
		}

		/*
		void ddOutput::writeFile(const std::string &filename, const std::string &outtype) const
		{
			this->write(filename, outtype);
		}

		void ddOutput::writeFile(const std::string &filename, const std::string &outtype) const
		{
			using namespace Ryan_Serialization;
			using namespace std;
			using boost::filesystem::path;
			string cmeth, uncompressed;

			string type = outtype;
			::rtmath::registry::IO_class_registry<ddOutput>::io_multi_type dllsaver = nullptr;
			
			Ryan_Serialization::uncompressed_name(filename, uncompressed, cmeth);
			path pext = path(uncompressed).extension();

			// Process dll hooks first
			auto hooks = usesDLLregistry<ddOutput_IO_output_registry,
				::rtmath::registry::IO_class_registry<ddOutput> >::getHooks();
			auto opts = registry::IO_options::generate();
			opts->filename(uncompressed);
			opts->filetype(type);
			for (const auto &hook : *hooks)
			{
				//if (hook.io_multi_matches(uncompressed.c_str(), type.c_str()))
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
				Ryan_Serialization::write<ddOutput>(*this, filename, "rtmath::ddscat::ddOutput");
			}
			else if (dllsaver)
			{
				// Most of these types aren't compressible or implement their
				// own compression schemes. So, it's not handled at this level.
				dllsaver(nullptr, opts, this);
				//dllsaver(filename.c_str(), this);
			} else {
				// Cannot match a file type to save.
				// Should never occur.
				RTthrow debug::xUnknownFileFormat(filename.c_str());
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
        */

		/** \brief Resize the orientation and fml tables
		*
		* The resizing operation is usually performed when data is loaded.
		* Resizing to a smaller size is typically quite destructive - ddOutputSingle
		* binds to a certain range of values for manipulations.
		*
		* \param numTotAngles is the total number of scattering angles considered, over 
		* all orientations. Used to allow heterogeneous angle combinations, from combined 
		* runs over the same particle.
		*
		* \param numOris is the number of orientation angles being considered.
		**/
		void ddOutput::resize(size_t numOris, size_t numTotAngles)
		{
			oridata_d.conservativeResize(numOris, Eigen::NoChange);
			oridata_i.conservativeResize(numOris, Eigen::NoChange);
			oridata_s.resize(numOris);
			ms.resize(numOris);
			numOriData = numOris;
			if (!fmldata) fmldata = boost::shared_ptr
				<Eigen::Matrix<float, Eigen::Dynamic, fmlColDefs::NUM_FMLCOLDEFS> >
				(new Eigen::Matrix<float, Eigen::Dynamic, fmlColDefs::NUM_FMLCOLDEFS>());
			if (numTotAngles) resizeFML(numTotAngles);
		}

		void ddOutput::resizeFML(size_t numTotAngles)
		{
			fmldata->conservativeResize(numTotAngles, Eigen::NoChange);
		}

		boost::shared_ptr<ddOutput> ddOutput::generate(const std::string &dir, bool noLoadRots)
		{
			boost::shared_ptr<ddOutput> res(new ddOutput());
			using namespace boost::filesystem;
			using boost::shared_ptr;
			using std::vector;

			path pBase(dir);
			if (!exists(pBase)) RTthrow debug::xMissingFile(dir.c_str());

			// Perform a single-level iteration through the path tree
			vector<path> cands;
			cands.reserve(50000);
			copy(directory_iterator(pBase), directory_iterator(), back_inserter(cands));

			// Iterate over and load each file. Loading files in parallel because each read operation is 
			// rather slow. The ddscat file parsers could use much improvement.

			//shared_ptr<ddPar> parfile;
			//shared_ptr<shapefile::shapefile> shape;
			std::mutex m_fmls, m_fmlmap, m_shape, m_par, m_other, m_pathlist, m_filecheck;

			// Pair up matching sca and fml files for a combined read.
			// Keys with only an sca entry are avg files.
			std::map<path, std::pair<path, path> > orisources;

			const size_t numThreads = rtmath::debug::getConcurrentThreadsSupported();
			std::vector<std::thread> pool;

			auto loadShape = [&](const path &p)
			{
				std::lock_guard<std::mutex> lock(m_shape);
				if (res->shape) return; // Only needs to be loaded once
				//if (noLoadRots) return;
				// Note: the hashed object is the fundamental thing here that needs to be loaded
				// The other stuff is only loaded for processing, and is not serialized directly.
				res->shape = boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile>
					(new ::rtmath::ddscat::shapefile::shapefile(p.string()));
				// Get the hash and load the stats
				//shapeHash = res->shape->hash();
				//if (dostats)
				//	res->stats = stats::shapeFileStats::genStats(res->shape);
			};
			auto loadPar = [&](const path &p)
			{
				std::lock_guard<std::mutex> lock(m_par);
				res->parfile = boost::shared_ptr<ddPar>(new ddPar(p.string()));
			};

			for (const auto &p : cands)
			{
				// Handle compressed files (in case my or Liu's scripts compressed the input)
				path praw;
				std::string meth;
				{
					//std::lock_guard<std::mutex> lock(m_filecheck);
					Ryan_Serialization::uncompressed_name(p, praw, meth);
				}
				// Extract entension of files in ._ form
				// Note: some files (like mtable) have no extension. I don't use these.
				if (!praw.has_extension()) continue;
				path pext = praw.extension();
				path pfileid = praw;
				pfileid.replace_extension();

				if ((pext.string() == ".sca" || pext.string() == ".fml") && noLoadRots) continue;
				if (pext.string() == ".sca" || pext.string() == ".fml" || pext.string() == ".avg")
				{
					if (!orisources.count(pfileid)) orisources[pfileid] = std::pair<path, path>(path(), path());
					if (pext.string() == ".sca" || pext.string() == ".avg")
					{
						orisources[pfileid].second = praw;
					} else {
						orisources[pfileid].first = praw;
					}
				} else if (praw.filename().string() == "ddscat.par") { // Match full name
					std::thread t(loadPar, praw);
					pool.push_back(std::move(t));
				} else if (praw.filename().string() == "shape.dat") { // Match full name
					/// \todo Add target.out loading and hashing for consistency verification
					//|| praw.filename().string() == "target.out") { // Match full name
					std::thread t(loadShape, praw);
					pool.push_back(std::move(t));
				}
			}

			std::vector<std::pair<path, path> > oris;
			oris.reserve(orisources.size());
			for (const auto &s : orisources)
				oris.push_back(s.second);
			res->resize(oris.size(), 0); // fml size is not yet known. These entries will be imported later.
			vector<ddOriData > fmls;
			fmls.reserve(orisources.size());
			size_t count = 0;

			auto process_path = [&](const std::pair<path, path> &p)
			{
				size_t mycount = count;
				{
					std::lock_guard<std::mutex> lock(m_fmls);
					++count;
				}
				ddOriData dat(*res, mycount, p.second.string(), p.first.string());
				{
					std::lock_guard<std::mutex> lock(m_fmlmap);
					fmls.push_back(std::move(dat));
				}
			};

			auto process_paths = [&]()
			{
				try {
					std::pair<path, path> p;
					for (;;)
					{
						{
							std::lock_guard<std::mutex> lock(m_pathlist);

							if (!oris.size()) return;
							p = oris.back();
							oris.pop_back();
						}

						process_path(p);
					}
				}
				catch (std::exception &e)
				{
					std::cerr << e.what() << std::endl;
					return;
				}
			};



			for (size_t i = 0; i<numThreads; i++)
			{
				std::thread t(process_paths);
				pool.push_back(std::move(t));
			}
			for (auto &t : pool)
				t.join();

			/// \todo Do table resorting for better memory access for both the orientation and fml tables.
			size_t numAngles = 0;
			for (const auto &f : fmls) // Avg files have no fml entries - only cross-sections are stored
				numAngles += f.numMat();
			res->resizeFML(numAngles);
			size_t i = 0;
			for (const auto &f : fmls)
			{
				f.doExportFMLs(i);
				i += f.numMat();
			}


			// Apply consistent generation
			//auto res = generate(avg, parfile, shape, fmls, scas);
			res->finalize();

			// Tag the source directory and ddscat version
			path pdir(dir);
			path pbdir = absolute(pdir);
			res->sources.insert(pbdir.string());

			return res;
		}


		void ddOutput::finalize()
		{
			using namespace boost::filesystem;
			using std::vector;

			if (shape)
			{
				shapeHash = shape->hash();
				// Stats can be requested at another time
				//stats = stats::shapeFileStats::genStats(shape);
			}

			// Pull the information from the first loaded entry
			/// \todo Pull the information from the first avg file?
			if (oridata_s.size())
			{
				const size_t _row = 0;
				auto &od = oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
				auto &os = oridata_s.at(_row);
				auto &oi = oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);

				ddOriData data(*this, _row);

				tags.insert(os.at(stat_entries::TARGET));
				// Extract the ddscat version from the target field
				// Find "ddscat/" and read until the next space
				ddUtil::getDDSCATbuild(os.at(stat_entries::TARGET), ddvertag);

				freq = data.freq();
				aeff = data.aeff();
				temp = data.guessTemp();
			}
			else {
				aeff = 0;
				freq = 0;
			}
			
			if (shape) {
				// Save the shape in the hash location, if necessary
				shape->writeToHash();
				// Resave the stats in the hash location
				//stats->writeToHash();
			}

			if (!ddvertag.size()) ddvertag = "unknown";
		}

		void ddOutput::expand(const std::string &outdir, bool writeShape) //const
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
			if (parfile)
				this->parfile->writeFile( (path(pOut)/"ddscat.par").string());

			// Write shape file
			if (writeShape)
			{
				if (!this->shape) this->loadShape();
				shape->writeFile( (path(pOut)/"target.out").string() );
			}
			/*
			auto oname = [](const boost::filesystem::path &base,
				boost::shared_ptr<ddOutputSingle> d) -> std::string
			{
				std::ostringstream n;
				/// \todo add a unique orientation hash
				n << d->beta() << "-" << d->theta() << 
					"-" << d->phi();
				return (base/path(n.str())).string();
			};
			*/
			auto onameb = [](const boost::filesystem::path &base,
				size_t i, size_t ni) -> std::string
			{
				using namespace std;
				ostringstream n;
				n << "w000r000"; //k";
				if (ni) {
					n << "k";
					n.width(ni);
					n.fill('0');
					n << i;
				}
				return (base/path(n.str())).string();
			};

			// Write fmls
			for (size_t i = 0; i < oridata_s.size(); ++i)
			{
				if (oridata_i(i, stat_entries::DOWEIGHT)) {
					std::string basename = onameb(pOut, i, 4);
					std::string fmlname = basename;
					std::string scaname = basename;
					fmlname.append(".fml");
					scaname.append(".sca");

					ddOriData obj(*this, i);
					obj.doImportFMLs();
					obj.writeFile(fmlname);
					obj.writeFile(scaname);
				} else {
					// File is an avg file of some sort
					std::string basename = onameb(pOut, i, 0);
					basename.append(".avg");

					ddOriData obj(*this, i);
					//obj.doImportFMLs();
					obj.writeFile(basename);
				}
			}

			/*
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
			*/
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
			// Will enable after actual use
			//if (!validateDir(pHashRuns)) RTthrow debug::xMissingFile(pHashRuns.string().c_str());
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

			float tDesc = (boost::math::round((float) temp*10.f)/10.f);
			//if (!tDesc) tDesc = (boost::math::round((float) ms.at(0).real() * 100000.f)/100000.f);


			std::ostringstream out;
			out << shapeHash.lower << "-"
				<< (boost::math::round((float) freq*10.f)/10.f) << "-"
				<< (boost::math::round((float) aeff*10.f)/10.f) << "-"
				<< tDesc << "-"
				<< oridata_s.size() << "-"
				<< rots.bN() << "-" << rots.tN() << "-" << rots.pN() << "-"
				<< ddvertag
				<< ".xml";

			res = out.str();
			return res;
		}

		std::string ddOutput::genNameSmall() const
		{
			std::string res;

			/* Name follows this pattern:
			 - frequency (to nearest tenth)
			 - aeff (to nearest tenth)
			 - number of sca matrices
			 - rotations
			 - ddscat version tag
			 */

			boost::filesystem::path p;
			rotations rots;
			parfile->getRots(rots);

			float tDesc = (boost::math::round((float) temp*10.f)/10.f);
			//if (!tDesc) tDesc = (boost::math::round((float) ms.at(0).real() * 100000.f)/100000.f);


			std::ostringstream out;
			out << (boost::math::round((float) freq*10.f)/10.f) << "-"
				<< (boost::math::round((float) aeff*10.f)/10.f) << "-"
				<< tDesc << "-"
				<< oridata_s.size() << "-"
				<< rots.bN() << "-" << rots.tN() << "-" << rots.pN() << "-"
				<< ddvertag;

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
		// TODO: RTthrow stuff?
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
