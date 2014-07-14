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
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cmath>
#include <ios>
#include <iomanip>
#include <thread>
#include <mutex>


#include <Ryan_Debug/debug.h>

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
#include "../rtmath/ddscat/dielTabFile.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace {
	boost::filesystem::path pHashRuns;
	//bool autoHashRuns = false;
	static bool isFMLforced = false;
	static bool forceWriteFML = true;
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

		void ddOutput::isForcingFMLwrite(bool& a, bool& b)
		{
			a = isFMLforced; b = forceWriteFML;
		}

		ddOutput::ddOutput() : 
			freq(0), aeff(0), temp(0), numOriData(0), ingest_rtmath_version(0)
		{
			resize(0, 0);
		}

		ddOutput::ddOutput(const ddOutput &src) :
			description(src.description),
			ingest_timestamp(src.ingest_timestamp),
			ingest_hostname(src.ingest_hostname),
			ingest_username(src.ingest_username),
			hostname(src.hostname),
			ingest_rtmath_version(src.ingest_rtmath_version),
			freq(src.freq),
			aeff(src.aeff),
			temp(src.temp),
			sources(src.sources),
			tags(src.tags),
			ddvertag(src.ddvertag),
			s(src.s),
			oridata_d(src.oridata_d),
			avgdata(src.avgdata),
			ms(src.ms),
			numOriData(src.numOriData),
			shapeHash(src.shapeHash),
			parsedShapeHash(src.parsedShapeHash),
			shape(src.shape),
			stats(src.stats)
		{
			fmldata = boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, fmlColDefs::NUM_FMLCOLDEFS> >
			(new Eigen::Matrix<float, Eigen::Dynamic, fmlColDefs::NUM_FMLCOLDEFS>
			(*(src.fmldata)));
			parfile = boost::shared_ptr<ddPar>(new ddPar(*(src.parfile)));
		}


		ddOutput::shared_data::shared_data() : version(0), num_dipoles(0), navg(0) {
			mins.fill(0); maxs.fill(0), TA1TF.fill(0); TA2TF.fill(0); LFK.fill(0);
			IPV1LF.fill(std::complex<double>(0, 0));
			IPV2LF.fill(std::complex<double>(0, 0));
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
		void ddOutput::loadShape(bool dostats) const
		{
			// Load stats and shape based on the hash
			shape = shapefile::shapefile::loadHash(shapeHash);
			if (!shape->hash().lower)
			{
				// Intelligently try to find the shape if not in the hash tree.
				// If the ddOutput sources field lists a reference directory, then use it to 
				// try and find a shapefile with a matching hash.
				if (sources.size())
				{
					using namespace boost::filesystem;
					path pBase(*(sources.begin()));
					path pshp = pBase / "shape.dat";
					if (exists(pshp))
					{
						boost::shared_ptr<shapefile::shapefile> scand(new shapefile::shapefile);
						scand->readFile(pshp.string());
						if (scand->hash() == shapeHash) shape = scand;
					}
				}
			}
			if (!shape->hash().lower) RTthrow debug::xMissingHash("shapefile", shapeHash.string().c_str());
			if (!dostats) return;
			stats = stats::shapeFileStats::genStats(shape);
			//stats = stats::shapeFileStats::loadHash(this->shapeHash);
			if (!stats) RTthrow debug::xMissingHash("shapestats", shapeHash.string().c_str()); // should never happen
			stats->load(shape);
		}

		boost::shared_ptr<const Eigen::MatrixXf> ddOutput::genWeights(
			boost::shared_ptr<weights::OrientationWeights3d> p) const
		{
			boost::shared_ptr<Eigen::MatrixXf> res(new Eigen::MatrixXf(oridata_d.rows(), 1));
			for (size_t i = 0; i < (size_t)res->rows(); ++i)
			{
				const double &beta = oridata_d(i, stat_entries::BETA);
				const double &theta = oridata_d(i, stat_entries::THETA);
				const double &phi = oridata_d(i, stat_entries::PHI);

				double wt = p->getWeight(beta, theta, phi);
				(*res)(i) = static_cast<float>(wt);
			}
			boost::shared_ptr<const Eigen::MatrixXf> resb(res);
			return resb;
		}

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
			avgdata.avg.conservativeResize(1, Eigen::NoChange);
			//oridata_i.conservativeResize(numOris, Eigen::NoChange);
			//oridata_s.resize(1);
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

		ddOutput::Avgdata::Avgdata() : beta_min(0), beta_max(0), beta_n(0),
			theta_min(0), theta_max(0), theta_n(0),
			phi_min(0), phi_max(0), phi_n(0), hasAvg(0)
		{
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
			auto loadShapeParsed = [&](const path &p)
			{
				std::lock_guard<std::mutex> lock(m_shape);
				if (res->parsedShapeHash.lower) return; // Only needs to be loaded once
				//if (noLoadRots) return;
				// Note: the hashed object is the fundamental thing here that needs to be loaded
				// The other stuff is only loaded for processing, and is not serialized directly.
				res->parsedShapeHash = boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile>
					(new ::rtmath::ddscat::shapefile::shapefile(p.string()))->hash();
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
			auto loadAvg = [&](const path &p)
			{
				std::lock_guard<std::mutex> lock(m_other);
				ddOriData dat(*res, p.string());
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
				if (pext.string() == ".sca" || pext.string() == ".fml")
				{
					if (!orisources.count(pfileid)) orisources[pfileid] = std::pair<path, path>(path(), path());
					if (pext.string() == ".sca")
					{
						orisources[pfileid].second = praw;
					} else {
						orisources[pfileid].first = praw;
					}
				} else if (pext.string() == ".avg") {
					std::thread t(loadAvg, praw);
					pool.push_back(std::move(t));
				} else if (praw.filename().string() == "ddscat.par") { // Match full name
					std::thread t(loadPar, praw);
					pool.push_back(std::move(t));
				} else if (praw.filename().string() == "shape.dat") { // Match full name
					std::thread t(loadShape, praw);
					pool.push_back(std::move(t));
				} else if (praw.filename().string() == "target.out") { // Match full name
					std::thread t(loadShapeParsed, praw);
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


			// Tag the source directory and ddscat version
			path pdir(dir);
			path pbdir = absolute(pdir);
			res->sources.insert(pbdir.string());

			// Apply consistent generation
			res->finalize();

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


			tags.insert(std::pair<std::string, std::string>("target", s.target));

			/*
			auto selectData = [&]() -> Eigen::Block<ddOutput::doubleType, 1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES, false, true>
			{
				if (avg(0)))
					return avg.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(0, 0);
				return oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(0, 0);
			};
			*/
			if (numOriData || avgdata.hasAvg)
			{
				//auto od = selectData();
				boost::shared_ptr<ddOriData> data;
				if (avgdata.hasAvg) data = boost::shared_ptr<ddOriData>(new ddOriData(*this, ""));
				else data = boost::shared_ptr<ddOriData>(new ddOriData(*this, 0));
			
				// Extract the ddscat version from the target field
				// Find "ddscat/" and read until the next space
				ddUtil::getDDSCATbuild(s.target, ddvertag);

				freq = data->freq();
				aeff = data->aeff();
				temp = data->guessTemp();
			} else if (parfile) {
				// select first wavelength and effective radius
				double min, max;
				size_t n;
				std::string spacing;
				parfile->getWavelengths(min, max, n, spacing);
				freq = units::conv_spec("um", "GHz").convert(min);
				parfile->getAeff(min, max, n, spacing);
				aeff = min;
				// attempt to load the dielectric file
				using namespace boost::filesystem;
				std::vector<std::string> diels;
				parfile->getDiels(diels);
				if (diels.size() && sources.size()) {
					std::string pardir = *(sources.begin());
					path ppar = path(pardir);
					//path ppar = path(filename).remove_filename();
					path pval(diels[0]);
					path prel = boost::filesystem::absolute(pval, ppar);
					if (boost::filesystem::exists(prel))
					{
						dielTab dt(prel.string());
						std::complex<double> diel;
						diel = dt.interpolate(freq);
						temp = refract::guessTemp(freq, diel);
					} else temp = 0;
				} else temp = 0;
			} else {
				aeff = 0;
				freq = 0;
				temp = 0;
			}
			
			if (shape) {
				// Save the shape in the hash location, if necessary
				//shape->writeToHash();
				// Resave the stats in the hash location
				//stats->writeToHash();
			}

			if (!ddvertag.size()) ddvertag = "unknown";
			ingest_hostname = Ryan_Debug::getHostname();
			ingest_username = Ryan_Debug::getUsername();
			using namespace boost::posix_time;
			using namespace boost::gregorian;
			ptime now = second_clock::local_time();
			ingest_timestamp = to_iso_string(now);
			ingest_rtmath_version = rtmath::debug::rev();
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
			for (size_t i = 0; i < (size_t) oridata_d.rows(); ++i)
			{
				//if (oridata_d(i, stat_entries::DOWEIGHT)) {
					std::string basename = onameb(pOut, i, 4);
					std::string fmlname = basename;
					std::string scaname = basename;
					fmlname.append(".fml");
					scaname.append(".sca");

					ddOriData obj(*this, i);
					obj.doImportFMLs();
					obj.writeFile(fmlname);
					obj.writeFile(scaname);
					/*
				} else {
					// File is an avg file of some sort
					std::string basename = onameb(pOut, i, 0);
					basename.append(".avg");

					ddOriData obj(*this, i);
					//obj.doImportFMLs();
					obj.writeFile(basename);
				}
					*/
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
				;

			config.add_options()
				;

			hidden.add_options()
				("ddOutput-force-write-fmls", po::value<bool>(), "Force / disable fml table writing")
				;
		}

		void ddOutput::process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;
			using boost::filesystem::path;

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

			if (vm.count("ddOutput-force-write-fmls"))
			{
				isFMLforced = true;
				forceWriteFML = vm["ddOutput-force-write-fmls"].as<bool>();
			}

		}

		/*
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
		*/


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
				<< oridata_d.rows() << "-"
				<< rots.bN() << "-" << rots.tN() << "-" << rots.pN() << "-"
				<< ddvertag;

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
				<< oridata_d.rows() << "-"
				<< rots.bN() << "-" << rots.tN() << "-" << rots.pN() << "-"
				<< ddvertag;

			res = out.str();
			return res;
		}

		std::string ddOutput::stat_entries::stringify(int val)
		{
#define _tostr(a) #a
#define tostr(a) _tostr(a)
#define check(a) if (val == a) return std::string( tostr(a) );
			check(D); //check(XMIN); check(XMAX); check(YMIN);
			//check(YMAX); check(ZMIN); check(ZMAX); 
			check(AEFF);
			check(WAVE); //check(FREQ); //check(NAMBIENT); check(TOL);
			//check(TA1TFX); check(TA1TFY); check(TA1TFZ); check(TA2TFX);
			//check(TA2TFY); check(TA2TFZ); 
			check(TFKX); check(TFKY);
			check(TFKZ); check(IPV1TFXR); check(IPV1TFXI); check(IPV1TFYR);
			check(IPV1TFYI); check(IPV1TFZR); check(IPV1TFZI); check(IPV2TFXR);
			check(IPV2TFXI); check(IPV2TFYR); check(IPV2TFYI); check(IPV2TFZR);
			check(IPV2TFZI); check(TA1LFX); check(TA1LFY); check(TA1LFZ);
			check(TA2LFX); check(TA2LFY); check(TA2LFZ); 
			//check(LFKX);
			//check(LFKY); check(LFKZ); check(IPV1LFXR); check(IPV1LFXI);
			//check(IPV1LFYR); check(IPV1LFYI); check(IPV1LFZR); check(IPV1LFZI);
			//check(IPV2LFXR); check(IPV2LFXI); check(IPV2LFYR); check(IPV2LFYI);
			//check(IPV2LFZR); check(IPV2LFZI); 
			check(BETA); check(THETA);
			check(PHI); //check(ETASCA); 
			check(QEXT1); check(QABS1);
			check(QSCA1); check(G11); check(G21); check(QBK1);
			check(QPHA1); check(QEXT2); check(QABS2); check(QSCA2);
			check(G12); check(G22); check(QBK2); check(QPHA2);
			check(QEXTM); check(QABSM); check(QSCAM); check(G1M);
			check(G2M); check(QBKM); check(QPHAM); check(QPOL);
			check(DQPHA); check(QSCAG11); check(QSCAG21); check(QSCAG31);
			check(ITER1); check(MXITER1); check(NSCA1); 
			check(QSCAG12);
			check(QSCAG22); check(QSCAG32); 
			check(ITER2); check(MXITER2); check(NSCA2); 
			check(QSCAG1M); check(QSCAG2M); check(QSCAG3M);
			//check(DOWEIGHT);
			return std::string("");
#undef _tostr
#undef tostr
#undef check
		}

		/*
		template<>
		std::string ddOutput::stat_entries::stringify<size_t>(int val)
		{
#define _tostr(a) #a
#define tostr(a) _tostr(a)
#define check(a) if (val == a) return std::string( tostr(a) );
			//check(VERSION); check(NUM_DIPOLES); check(NAVG); check(DOWEIGHT);
			return std::string("");
#undef _tostr
#undef tostr
#undef check
		}
		

		template<>
		std::string ddOutput::stat_entries::stringify<std::string>(int val)
		{
#define _tostr(a) #a
#define tostr(a) _tostr(a)
#define check(a) if (val == a) return std::string( tostr(a) );
			check(TARGET); check(DDAMETH); check(CCGMETH); check(SHAPE);
			return std::string("");
#undef _tostr
#undef tostr
#undef check
		}
		*/

		std::string ddOutput::fmlColDefs::stringify(int val)
		{
#define _tostr(a) #a
#define tostr(a) _tostr(a)
#define check(a) if (val == a) return std::string( tostr(a) );
			check(ORIINDEX); check(THETAB); check(PHIB);
			check(F00R); check(F00I); check(F01R); check(F01I);
			check(F10R); check(F10I); check(F11R); check(F11I);
			return std::string("");
#undef _tostr
#undef tostr
#undef check
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
